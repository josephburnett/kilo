# Kilo's SD card on the laptop (Debian in WSL: the card reader belongs to
# Windows, docs/laptop-setup.md "Windows and macOS"). Both ask for the sudo
# password. Another drive letter: make mount DRIVE=F
DRIVE ?= E
CARD  ?= /mnt/$(shell echo $(DRIVE) | tr A-Z a-z)

.PHONY: mount unmount

# Makes the card in Windows drive $(DRIVE): readable at $(CARD).
mount:
	@# a mount left over from a card that was taken out answers "No such device": drop it first
	@if grep -q ' $(CARD) ' /proc/mounts && ! ls $(CARD) >/dev/null 2>&1; then echo "dropping the stale mount at $(CARD)"; sudo umount -l $(CARD); fi
	test -d $(CARD) || sudo mkdir -p $(CARD)
	mountpoint -q $(CARD) || sudo mount -t drvfs $(DRIVE): $(CARD)
	@test -f $(CARD)/kernel7.img && echo "Kilo's card is at $(CARD)" || echo "$(CARD) is mounted, but has no kernel7.img: is drive $(DRIVE): Kilo's card?"

# Releases it. Afterwards eject the card from Windows (the tray's "Safely Remove").
unmount:
	sudo umount -l $(CARD)
	@echo "released: now eject drive $(DRIVE): from Windows"
