# The build id, for kernel/Makefile (KILO_BUILD) and kernel/host/Makefile
# (KILO_MIND_BUILD): the first 8 hex digits of the commit, as for the body
# (-DKILO_BUILD=0x<hash>, mind/kilo/tools/flash.py). Set BUILD_ID_FILE
# before including: it is rewritten only when the hash changes, so what
# depends on it is rebuilt only then.
BUILD_ID := $(shell git rev-parse --short=8 HEAD 2>/dev/null || echo 0)
$(shell mkdir -p $(dir $(BUILD_ID_FILE)) && (echo $(BUILD_ID) | cmp -s - $(BUILD_ID_FILE) || echo $(BUILD_ID) > $(BUILD_ID_FILE)))
