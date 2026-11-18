// CAL: joint calibration over the host channel (host_server.h's header, the
// spec's CAL / CAL_STATUS). The body keeps the calibration (CAL_GET answers
// CAL_VALUES; CAL_SET changes one offset in RAM, and every pulse uses it from
// the next tick, so a held pose moves at once; CAL_SAVE writes it to the
// EEPROM slot, EVENT EEPROM_SAVED when written; CAL_REVERT reloads the slot).
// The mind only relays, through the link's one command slot:
//
//   Send     the op's body command (GET: CAL_GET at once) when the slot is free
//   Wait     its ACK; SAVE then waits for EEPROM_SAVED (Saved)
//   Get      CAL_GET, to answer with the offsets as they are now
//   GetWait  CAL_VALUES -> CAL_STATUS to the client
//
// The slot keeps one answer until the next request, and the mind's other
// users (the trick player, the walker's SET_STREAM, STATS) ask in the same
// pass after link.poll(), so the answer is taken in collect(), right after
// link.poll(). A slot that stays busy for 2 s, or a body that does not
// answer, fails the CAL.
#include "link/names.h"
#include "net/host_server.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

constexpr uint64_t kSlotWaitUs = 2000000;  // the command slot busy this long: BUSY
constexpr uint64_t kSaveWaitUs = KILO_HOST_CAL_SAVE_WAIT_MS * 1000u;
constexpr int16_t kOffsetMax = 400;  // the body's CAL_SET range, +-25 deg (a16)

const char* op_name(uint8_t op) {
  switch (op) {
    case KILO_CAL_OP_SET: return "set";
    case KILO_CAL_OP_SAVE: return "save";
    case KILO_CAL_OP_REVERT: return "revert";
    default: return "get";
  }
}

// "+8.50": an a16 angle in degrees, two decimals.
void deg(char* out, uint32_t n, int16_t a16) {
  int32_t v = (int32_t)a16 * 100 / 16;
  int32_t a = v < 0 ? -v : v;
  format(out, n, "%s%d.%02d", v < 0 ? "-" : "+", (int)(a / 100), (int)(a % 100));
}

}  // namespace

void HostServer::cal_request(uint32_t i, uint8_t seq, const uint8_t* p) {
  const uint8_t type = KILO_HOST_MSG_CAL;
  kilo_host_cal_t q;
  mem_copy(&q, p, sizeof q);
  if (q.op > KILO_CAL_OP_REVERT) return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "op: 0 get, 1 set, 2 save, 3 revert");
  if (q.op == KILO_CAL_OP_SET && q.joint >= KILO_CHANNEL_COUNT)
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "joint: 0..15");
  if (q.op == KILO_CAL_OP_SET && (q.offset_a16 < -kOffsetMax || q.offset_a16 > kOffsetMax))
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG,
                  "offset outside -25..+25 deg (the body's CAL_SET range)");
  if (q.op != KILO_CAL_OP_GET) {
    const Goal* g = link_.goal(KILO_MSG_SET_POSTURE);
    bool calib = g != nullptr && g->len >= 2 && g->payload[1] == KILO_POSTURE_CALIB;
    if (controller_ != (int32_t)i && !calib)
      return result(i, seq, type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER,
                    "take control, or hold the calibration posture (kilo posture calib --host)");
    const char* why = cal_gate_ != nullptr ? cal_gate_->cal_refusal() : nullptr;
    if (why != nullptr) return result(i, seq, type, KILO_HOST_RESULT_CODE_BUSY, why);
  }
  if (bridge_) return result(i, seq, type, KILO_HOST_RESULT_CODE_BRIDGED, "the bridge is open");
  if (flashing()) return result(i, seq, type, KILO_HOST_RESULT_CODE_FLASHING, "the body is being flashed");
  if (updating()) return result(i, seq, type, KILO_HOST_RESULT_CODE_UPDATING, "the kernel is being updated");
  if (!link_.connected() || !link_.state().has_hello || link_.state().proto_ok == 0)
    return result(i, seq, type, KILO_HOST_RESULT_CODE_NO_BODY, "no body to ask (no HELLO yet)");
  if (cal_.step != CalStep::None)
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BUSY, "another calibration request is in flight");
  cal_.client = (int32_t)i;
  cal_.seq = seq;
  cal_.op = q.op;
  cal_.joint = q.joint;
  cal_.offset_a16 = q.offset_a16;
  cal_.step = CalStep::Send;
  cal_.saved = false;
  cal_.deadline_us = clock_.now_us() + kSlotWaitUs;
  if (q.op == KILO_CAL_OP_SET) {
    char d[16];
    deg(d, sizeof d, q.offset_a16);
    logf(log_, LogLevel::Info, "host: client %d: calibration joint %u offset %s deg", (int)i, (unsigned)q.joint, d);
  } else if (q.op != KILO_CAL_OP_GET) {
    logf(log_, LogLevel::Info, "host: client %d: calibration %s", (int)i, op_name(q.op));
  }
  cal_step(clock_.now_us());
}

void HostServer::cal_fail(uint8_t code, const char* why) {
  logf(log_, LogLevel::Warn, "host: calibration %s failed: %s", op_name(cal_.op), why);
  if (cal_.client >= 0 && clients_[cal_.client].open) result((uint32_t)cal_.client, cal_.seq, KILO_HOST_MSG_CAL, code, why);
  cal_.step = CalStep::None;
  cal_.client = -1;
}

void HostServer::on_body_event(const kilo_event_t& e) {
  // (it may come in the same link.poll() as CAL_SAVE's ACK, before collect() has seen the ACK)
  if (e.code != KILO_EVENT_EEPROM_SAVED || cal_.op != KILO_CAL_OP_SAVE) return;
  if (cal_.step != CalStep::Wait && cal_.step != CalStep::Saved) return;
  cal_.saved = true;
  if (cal_.step == CalStep::Saved) {
    cal_.step = CalStep::Get;
    cal_.deadline_us = clock_.now_us() + kSlotWaitUs;
  }
}

void HostServer::collect() { cal_step(clock_.now_us()); }

void HostServer::cal_step(uint64_t now) {
  if (cal_.step == CalStep::None) return;
  if (!link_.connected() || bridge_ || link_paused()) return cal_fail(KILO_HOST_RESULT_CODE_NO_BODY, "the link went away");
  const Request& r = link_.req();
  // by type: a query's retries take fresh seqs, and nobody else sends CAL_* (or can
  // request at all while ours waits; collect() takes the answer before anyone could)
  bool ours = r.type == cal_.req_type;
  switch (cal_.step) {
    case CalStep::Send:
    case CalStep::Get: {
      uint8_t t = KILO_MSG_CAL_GET;
      uint8_t payload[3] = {0, 0, 0};
      uint8_t len = 0;
      if (cal_.step == CalStep::Send) {
        if (cal_.op == KILO_CAL_OP_SET) {
          t = KILO_MSG_CAL_SET;
          kilo_cal_set_t c = {cal_.joint, cal_.offset_a16};
          mem_copy(payload, &c, sizeof c);
          len = sizeof c;
        } else if (cal_.op == KILO_CAL_OP_SAVE) {
          t = KILO_MSG_CAL_SAVE;
        } else if (cal_.op == KILO_CAL_OP_REVERT) {
          t = KILO_MSG_CAL_REVERT;
        }
      }
      if (r.state == RequestState::Waiting || !link_.request(t, payload, len)) {
        if (now >= cal_.deadline_us) cal_fail(KILO_HOST_RESULT_CODE_BUSY, "the body's command slot stayed busy");
        return;
      }
      cal_.req_type = t;
      cal_.req_seq = link_.req().seq;
      cal_.step = t == KILO_MSG_CAL_GET ? CalStep::GetWait : CalStep::Wait;
      return;
    }
    case CalStep::Wait:
      if (!ours) return cal_fail(KILO_HOST_RESULT_CODE_BUSY, "the command slot was taken");
      if (r.state == RequestState::Waiting) return;
      if (r.state == RequestState::TimedOut) return cal_fail(KILO_HOST_RESULT_CODE_NO_BODY, "the body did not answer");
      if (r.status != KILO_ACK_STATUS_OK) {
        char why[64];
        format(why, sizeof why, "the body refused %s: %s", msg_name(cal_.req_type), ack_status_name(r.status));
        return cal_fail(r.status == KILO_ACK_STATUS_EEPROM_BUSY ? KILO_HOST_RESULT_CODE_BUSY : KILO_HOST_RESULT_CODE_BAD_ARG,
                        why);
      }
      cal_.step = cal_.op == KILO_CAL_OP_SAVE && !cal_.saved ? CalStep::Saved : CalStep::Get;
      cal_.deadline_us = now + (cal_.step == CalStep::Saved ? kSaveWaitUs : kSlotWaitUs);
      return;
    case CalStep::Saved:
      if (now < cal_.deadline_us) return;
      logf(log_, LogLevel::Warn, "host: calibration save: no EEPROM_SAVED within %u ms (queued only)",
           (unsigned)KILO_HOST_CAL_SAVE_WAIT_MS);
      cal_.step = CalStep::Get;
      cal_.deadline_us = now + kSlotWaitUs;
      return;
    case CalStep::GetWait: {
      if (!ours) return cal_fail(KILO_HOST_RESULT_CODE_BUSY, "the command slot was taken");
      if (r.state == RequestState::Waiting) return;
      if (r.state == RequestState::TimedOut) return cal_fail(KILO_HOST_RESULT_CODE_NO_BODY, "the body did not answer CAL_GET");
      if (r.status != KILO_ACK_STATUS_OK || r.reply_len != sizeof(kilo_cal_values_t))
        return cal_fail(KILO_HOST_RESULT_CODE_NO_BODY, "no CAL_VALUES from the body");
      kilo_host_cal_status_t st;
      mem_zero(&st, sizeof st);
      st.op = cal_.op;
      st.saved = cal_.saved ? 1 : 0;
      mem_copy(st.offset, r.reply, sizeof st.offset);
      if (cal_.op == KILO_CAL_OP_SAVE)
        logf(log_, LogLevel::Info, "host: calibration saved%s", cal_.saved ? " (EEPROM_SAVED)" : " (queued, not confirmed)");
      if (cal_.client >= 0 && clients_[cal_.client].open)
        reply((uint32_t)cal_.client, cal_.seq, &st, sizeof st, KILO_HOST_MSG_CAL_STATUS);
      cal_.step = CalStep::None;
      cal_.client = -1;
      return;
    }
    case CalStep::None:
      return;
  }
}

}  // namespace kilo
