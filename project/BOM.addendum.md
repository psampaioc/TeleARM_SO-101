# Project-specific BOM addendum

> **PROJECT ADDITION.** This file is not the official BOM. The unchanged upstream BOM is `../upstream/BOM.original.md`.

## Decisions

| Upstream item | Our decision | Reason | Validation status |
|---|---|---|---|
| `STS3215 Servo 7.4V, 1/345 gear (C001)` | Use six 12 V follower C018 servos | Higher-torque option selected for this build | Purchased; receipt confirms six `STS3215-C018`, 12 V, 30 kg·cm, 1/345. Check physical labels before first power-up. |
| `Power Supply` | Use regulated 12 V, minimum 5 A, for the C018 follower | Must match the selected servo voltage and current | User reports having a 12 V/5 A supply; verify its label, connector, polarity, and controller compatibility before connecting. |
| Parts for leader arm | Exclude | We are building follower only | Project decision |
| Camera/AI additions | Defer | Avoid changing the validated first build | Project decision |

## Do not infer

- Do not treat this addendum as an official BOM.
- Do not replace the upstream controller with an ESP32-only design before electrical validation.
- Do not buy a generic servo, power supply, USB cable, clamp or controller based only on a category name.

## Purchase record

The C018 motors were purchased from Shenzhen Feite Model Co., Ltd.; the local receipt is `../records/hardware/servo-purchase-receipt.pdf`. The pre-purchase comparison is closed for this build. Preserve the upstream links and prices in `../upstream/BOM.original.md` as source history.
