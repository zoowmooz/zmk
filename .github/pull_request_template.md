<!-- If you're adding a board/shield please fill out this check-list, otherwise you can delete it -->
## Board/Shield Check-list
 - [ ] This board/shield is tested working on real hardware
 - [ ] Definitions follow the general style of other shields/boards upstream ([Reference](https://zmk.dev/docs/development/new-shield))
 - [ ] `.zmk.yml` metadata file added
 - [ ] Proper Copyright + License headers added to applicable files
 - [ ] General consistent formatting of DeviceTree files
 - [ ] Keymaps do not use deprecated key defines (Check using the [upgrader tool](https://zmk.dev/docs/codes/keymap-upgrader))
 - [ ] `&pro_micro` used in favor of `&pro_micro_d/a` if applicable
 - [ ] If split, no name added for the right/peripheral half
 - [ ] `.conf` file has optional extra features commented out
