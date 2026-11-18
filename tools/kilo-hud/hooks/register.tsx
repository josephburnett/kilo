import { atom, read, update } from 'claude-code'
import type { EngineInterface, Register } from 'claude-code'

import type { Hud } from '../types'

const PANE = 'kilo-hud'
const POLL_MS = 20000

const EMPTY: Hud = {
  owner: 'unknown',
  ownerAt: 0,
  isFound: false,
  address: '',
  mindBuild: '',
  checkedAt: 0,
  mode: '',
  batteryMv: 0,
  firmware: '',
  statusAt: 0,
  statusError: '',
  repoHead: '',
}

const hud = atom({ plugin: 'kilo-hud', key: 'hud' } as const, EMPTY, {
  shape: 'hud-1',
})

type Found = { address: string; build: string | null }

const firstMatch = (text: string, pattern: RegExp): string => {
  const hit = pattern.exec(text)

  return hit === null ? '' : (hit[1] ?? '').trim()
}

const ago = (now: number, then: number): string => {
  if (then === 0) {
    return 'never'
  }

  const seconds = Math.max(0, Math.round((now - then) / 1000))

  if (seconds < 90) {
    return `${seconds} s ago`
  }

  return `${Math.round(seconds / 60)} min ago`
}

const isUpdateReady = (state: Hud): boolean =>
  state.isFound &&
  state.mindBuild !== '' &&
  state.repoHead !== '' &&
  state.mindBuild !== state.repoHead

const oneLine = (state: Hud): string => {
  if (!state.isFound) {
    const said = state.owner === 'unknown' ? '' : ` (you said: ${state.owner})`

    return `Kilo: not on the network${said}`
  }

  const battery = state.batteryMv > 0 ? ` · ${state.batteryMv} mV` : ''
  const update = isUpdateReady(state) ? ' · update ready' : ''

  return `Kilo: on at ${state.address}${battery}${update}`
}

let isBusy = false
let repo = ''

// The repository is the one the session runs in; its virtualenv holds the kilo command.
async function findRepo($: EngineInterface): Promise<string> {
  if (repo === '') {
    try {
      const top = await $.process.run(['git', 'rev-parse', '--show-toplevel'], { timeoutMs: 5000 })

      if (top.exitCode === 0) {
        repo = top.stdout.trim()
      }
    } catch {
      repo = ''
    }
  }

  return repo
}

// The name lookup is a multicast question: it opens no connection to Kilo,
// so it does not wake him from light sleep. The full status does (it
// connects), so it is read only when he appears or when asked for.
async function poll($: EngineInterface, isStatusWanted: boolean): Promise<Hud> {
  if (isBusy) {
    return read($, hud)
  }

  isBusy = true

  try {
    const now: number = await $.clock.now()
    const before: Hud = await read($, hud)
    const REPO = await findRepo($)
    const KILO = `${REPO}/.venv/bin/kilo`
    let found: Found | null = null

    try {
      const looked = await $.process.run([KILO, 'find', '--json', '--timeout', '1.5'], {
        timeoutMs: 15000,
      })

      if (looked.exitCode === 0) {
        const list = JSON.parse(looked.stdout) as Found[]
        found = list.find(one => typeof one.address === 'string') ?? null
      }
    } catch {
      found = null
    }

    let repoHead = before.repoHead

    try {
      const head = await $.process.run(['git', '-C', REPO, 'rev-parse', '--short=8', 'HEAD'], {
        timeoutMs: 5000,
      })

      if (head.exitCode === 0) {
        repoHead = head.stdout.trim()
      }
    } catch {
      repoHead = before.repoHead
    }

    let next: Hud = {
      ...before,
      isFound: found !== null,
      address: found === null ? before.address : found.address,
      mindBuild: found === null ? before.mindBuild : (found.build ?? before.mindBuild),
      checkedAt: now,
      repoHead,
    }

    const isNew = found !== null && (!before.isFound || before.address !== found.address)

    if (found !== null && (isStatusWanted || isNew)) {
      try {
        const status = await $.process.run([KILO, '--host', found.address, 'status'], {
          timeoutMs: 20000,
        })

        if (status.exitCode === 0) {
          const text = status.stdout
          const battery = Number(firstMatch(text, /^battery\s+(\d+) mV/m))

          next = {
            ...next,
            mindBuild: firstMatch(text, /^mind\s+build (\w+)/m) || next.mindBuild,
            mode: firstMatch(text, /^mode\s+(\w+)/m),
            batteryMv: Number.isFinite(battery) ? battery : 0,
            firmware: firstMatch(text, /^firmware\s+(.*)$/m),
            statusAt: now,
            statusError: '',
          }
        } else {
          next = {
            ...next,
            statusError: (status.stderr || status.stdout).trim().split('\n').slice(-1)[0] ?? 'status failed',
          }
        }
      } catch {
        next = { ...next, statusError: 'status did not answer in time' }
      }
    }

    await update($, hud, () => next)
    $.ui.status(oneLine(next))

    if (before.checkedAt !== 0 && before.isFound !== next.isFound) {
      $.ui.toast(next.isFound ? `Kilo is on the network at ${next.address}` : 'Kilo left the network')
    }

    return next
  } finally {
    isBusy = false
  }
}

function summary(state: Hud, now: number): string {
  const lines = [oneLine(state)]

  if (state.isFound) {
    lines.push(
      `mind build ${state.mindBuild || '?'}, repository ${state.repoHead || '?'}: ${
        isUpdateReady(state) ? 'an update is ready to install' : 'up to date'
      }`,
    )

    if (state.statusAt !== 0) {
      lines.push(
        `mode ${state.mode || '?'}, battery ${state.batteryMv || '?'} mV, body firmware: ${
          state.firmware || '?'
        } (read ${ago(now, state.statusAt)})`,
      )
    }

    if (state.statusError !== '') {
      lines.push(`status: ${state.statusError}`)
    }
  }

  return lines.join('\n')
}

export const register: Register = on => {
  on('session.start', async ($, e, next) => {
    await $.command.register({
      name: 'kilo',
      description: "Kilo's status pane; /kilo up, /kilo down tell Claude whether he is powered and connected",
      argumentHint: '[up | down | refresh | close]',
    })
    void poll($, false)
    $.clock.every(POLL_MS, () => {
      void poll($, false)
    })

    return next(e)
  })

  on('command.run', { command: 'kilo' }, async ($, e) => {
    const word = e.args.trim().toLowerCase()
    const now = await $.clock.now()

    if (word === 'close') {
      await $.ui.close({ id: PANE })

      return { text: 'Kilo pane closed. The status line stays.' }
    }

    if (word === 'up' || word === 'on') {
      await update($, hud, state => ({ ...state, owner: 'up' as const, ownerAt: now }))
      const state = await poll($, true)
      void $.prompt.submit({
        text: `/kilo up: Kilo is powered on with the network cable in.\n${summary(state, now)}`,
        asUser: true,
      })

      return { text: 'Told Claude that Kilo is powered on and connected.' }
    }

    if (word === 'down' || word === 'off') {
      await update($, hud, state => ({ ...state, owner: 'down' as const, ownerAt: now }))
      const state = await poll($, false)

      return {
        text: `Noted for Claude: Kilo is switched off or unplugged. Do not try to reach him until /kilo up.\n${oneLine(state)}`,
      }
    }

    if (word !== '' && word !== 'refresh') {
      return { text: 'Use /kilo, /kilo up, /kilo down, /kilo refresh or /kilo close.' }
    }

    await $.ui.open({ id: PANE, title: 'Kilo' })
    const state = await poll($, true)

    return { text: summary(state, now) }
  })

  on('ui.render', { component: 'Pane', requestId: PANE }, async ($, e) => {
    const { Box, Text } = $.ui.resolve(e)
    const state = await read($, hud)
    const now = await $.clock.now()
    const said =
      state.owner === 'unknown'
        ? 'nothing yet (/kilo up or /kilo down)'
        : `${state.owner === 'up' ? 'on, cable in' : 'off or unplugged'} (${ago(now, state.ownerAt)})`

    return (
      <Box flexDirection="column">
        <Text bold>{state.isFound ? 'ON THE NETWORK' : 'NOT ON THE NETWORK'}</Text>
        <Text dimColor>looked {ago(now, state.checkedAt)}, every 20 s</Text>
        <Text> </Text>
        <Text>You said: {said}</Text>
        <Text>Address: {state.address || '?'}{state.isFound ? '' : ' (last seen)'}</Text>
        <Text> </Text>
        <Text bold>Mind (the Pi)</Text>
        <Text>on Kilo: {state.mindBuild || '?'}</Text>
        <Text>in the repository: {state.repoHead || '?'}</Text>
        <Text>
          {state.mindBuild === '' || state.repoHead === ''
            ? 'update: unknown'
            : state.mindBuild === state.repoHead
              ? 'up to date'
              : 'UPDATE READY to install'}
        </Text>
        <Text> </Text>
        <Text bold>Last full reading</Text>
        {state.statusAt === 0 ? (
          <Text dimColor>none yet (/kilo refresh)</Text>
        ) : (
          <Box flexDirection="column">
            <Text>mode: {state.mode || '?'}</Text>
            <Text>battery: {state.batteryMv > 0 ? `${state.batteryMv} mV` : '?'}</Text>
            <Text>body: {state.firmware || '?'}</Text>
            <Text dimColor>read {ago(now, state.statusAt)}</Text>
          </Box>
        )}
        {state.statusError !== '' && <Text dimColor>status: {state.statusError}</Text>}
        <Text> </Text>
        <Text dimColor>/kilo up · /kilo down · /kilo refresh · /kilo close</Text>
      </Box>
    )
  })
}
