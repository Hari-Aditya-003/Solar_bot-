#!/bin/bash
# Fix UART permissions for RPi 5 — run once with sudo
# Usage: sudo bash fix_uart_permissions.sh

echo "=== Fixing UART permissions for RP3 V2 ==="

# 1. Create persistent udev rule so ttyAMA10 is always accessible by dialout group
RULES_FILE="/etc/udev/rules.d/99-uart-rp3v2.rules"
echo 'KERNEL=="ttyAMA10", GROUP="dialout", MODE="0660"' | sudo tee $RULES_FILE
echo 'KERNEL=="ttyAMA0",  GROUP="dialout", MODE="0660"' | sudo tee -a $RULES_FILE
echo "  Created: $RULES_FILE"

# 2. Apply immediately without reboot
sudo chmod 660 /dev/ttyAMA10 2>/dev/null && echo "  Fixed:   /dev/ttyAMA10  (660, dialout)"
sudo chown root:dialout /dev/ttyAMA10 2>/dev/null

sudo chmod 660 /dev/ttyAMA0 2>/dev/null && echo "  Fixed:   /dev/ttyAMA0   (660, dialout)"
sudo chown root:dialout /dev/ttyAMA0 2>/dev/null

# 3. Reload udev
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "  udev rules reloaded"

echo ""
echo "=== Done — verify with: ls -la /dev/ttyAMA10 /dev/ttyAMA0 ==="
ls -la /dev/ttyAMA10 /dev/ttyAMA0 2>/dev/null
