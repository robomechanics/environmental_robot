#!/usr/bin/env python3

import hebi
from time import sleep

lookup = hebi.Lookup()

sleep(2)
print('Modules found on network:')
for entry in lookup.entrylist:
	print(f'{entry.family} | {entry.name}')
