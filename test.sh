#!/bin/bash
#https://nickdesaulniers.github.io/blog/2017/05/16/submitting-your-first-patch-to-the-linux-kernel-and-responding-to-feedback/
make && (sudo rmmod msi-psu || true) && sudo insmod ./msi-psu.ko && time sensors msipsu-*