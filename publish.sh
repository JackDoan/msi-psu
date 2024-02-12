#!/bin/bash
cp msi-psu.c ../linux/drivers/hwmon/
cp README.rst ../linux/Documentation/hwmon/msi-psu.rst

cd ../linux
git add drivers/hwmon/msi-psu.c Documentation/hwmon/msi-psu.rst
git status

exit 0
git commit --amend

git send-email \
--to='Jack Doan <me@jackdoan.com>' \
--cc-cmd='./scripts/get_maintainer.pl --norolestats v2-0001-hwmon-Add-MSI-PSU-HID-monitoring-driver.patch' \
--in-reply-to 20240108185604.2858930-1-me@jackdoan.com \
v2-0001-hwmon-Add-MSI-PSU-HID-monitoring-driver.patch