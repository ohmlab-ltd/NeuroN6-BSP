# Security policy

See also [neuron6.com/security](https://neuron6.com/security).

## Reporting a vulnerability

- Email security@ohmlab.co.uk with a description, the affected versions and
  steps to reproduce. Do not open a public issue.
- Reports are acknowledged within 5 working days, with a status update at
  least monthly until resolved.

## Support

Security updates until 31 December 2031, announced on GitHub.

## Scope

The drivers, the libraries under `Middlewares/NeuroN6`, the examples, and the
boot loader's source and image in `FSBL/`.

## Limits

- Firmware images are checked by CRC, not signed: corruption is detected,
  tampering is not.
- No network update path. Updates are installed over USB.
- Model weights in external flash are neither encrypted nor authenticated.
- The USB serial port and vendor endpoint are not authenticated. A host on
  either can reboot the board into the boot loader, and the vendor endpoint
  gives it the board's access point password.
- Wi-Fi: WPA2 and WPA3 only. The board's access point uses a random password
  from the chip's random number generator, stored on the board. The video link
  runs on that access point only, encrypted by Wi-Fi alone; a link key can
  authenticate the host.
- BLE: no characteristic in the examples can be written over an unencrypted
  link.
