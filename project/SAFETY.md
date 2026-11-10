# Project safety addendum

> **PROJECT ADDITION.** Follow upstream assembly and safety guidance first. This file adds project gates only.

- Never power an unverified servo/controller combination.
- Verify voltage, polarity, connector and current limit with a meter and datasheet before connection.
- Use a local power cut-off before the first motion test.
- Test without payload, at low speed, with the arm restrained and clear of people.
- Treat stall torque as a damage limit, not as rated payload.
- Do not energize a soldering iron or use the arm for hot work in the first build.
- For the six 12 V STS3215-C018 servos, the upstream BOM specifies a regulated 12 V supply rated at least 5 A. The user reports having a 12 V/5 A supply; verify its label, connector, polarity, and controller compatibility before connecting it.
