# Third-Party Notices

This file is an attribution notice for third-party software statically
embedded in the prebuilt TDK library (`libflexiv_tdk.so` on Linux,
`libflexiv_tdk.dylib` on macOS). It does **not** change the license of
Flexiv TDK, does **not** grant rights in Flexiv trademarks, and does
**not** make this SDK an Eclipse Foundation project.

## Flexiv TDK

Flexiv-authored TDK code is licensed under the Apache License 2.0
(see `LICENSE`). Recipients do **not** receive TDK under the Eclipse
Public License or under MIT.

## License election for Eclipse Zenoh

`flexiv_tdk_client` builds **Eclipse zenoh-c 1.9.0** and **zenoh-cpp 1.9.0**
from the official Eclipse repositories and statically links them (with
symbols hidden) into `libflexiv_tdk`.

Upstream licenses those components under a **dual license**:
Apache License 2.0 **OR** Eclipse Public License 2.0
(SPDX: `EPL-2.0 OR Apache-2.0`). There is no separate Apache-only
source tree; the dual-licensed official 1.9.0 tags are what TDK uses.

**Flexiv Robotics elects Apache License 2.0 as the only license** under
which that Zenoh code is redistributed in this SDK.

- Recipients receive the embedded Zenoh code under Apache-2.0, not EPL-2.0.
- Flexiv does not sublicense Zenoh under EPL-2.0.
- Flexiv does not offer EPL source-code, copyleft, or commercial-distributor
  obligations for this SDK.
- `licenses/zenoh.LICENSE` contains the **elected Apache-2.0 text only**.
  Do not treat this SDK as an EPL distribution.

## Embedded in `libflexiv_tdk`

| Component | Version | License in this SDK | Upstream |
| --------- | ------- | ------------------- | -------- |
| [spdlog](https://github.com/gabime/spdlog) | 1.9.2 | MIT | https://github.com/gabime/spdlog/tree/v1.9.2 |
| [fmt](https://github.com/fmtlib/fmt) (bundled by spdlog) | 8.0.1 | MIT | https://github.com/fmtlib/fmt |
| [Eclipse zenoh-c](https://github.com/eclipse-zenoh/zenoh-c) | 1.9.0 | **Apache-2.0 (elected)** | https://github.com/eclipse-zenoh/zenoh-c/tree/1.9.0 |
| [Eclipse zenoh-cpp](https://github.com/eclipse-zenoh/zenoh-cpp) | 1.9.0 | **Apache-2.0 (elected)** | https://github.com/eclipse-zenoh/zenoh-cpp/tree/1.9.0 |

Copyright notices retained as required by those licenses:

- spdlog: Copyright (c) 2016 Gabi Melman
- fmt: Copyright (c) 2012 - present, Victor Zverovich
- Zenoh: Copyright (c) 2017, 2022 ZettaScale Technology; Copyright (c) Eclipse Zenoh project contributors

Full license texts:

- `licenses/spdlog.LICENSE`
- `licenses/fmt.LICENSE`
- `licenses/zenoh.LICENSE` (Apache-2.0 as elected by Flexiv)

zenoh-c is the official Eclipse C binding and may statically include
Rust crates and cryptographic support libraries that Zenoh 1.9.0
depends on. Those transitive components are typically under Apache-2.0,
MIT, BSD, ISC, or similar permissive licenses. They are not GPL or LGPL.


## Disclaimer and trademarks

Third-party software is provided by its authors on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND. Flexiv warranties,
indemnities, and support (if any) in a commercial agreement cover
**Flexiv-authored TDK code only**, unless that agreement expressly
says otherwise.

"Eclipse", "Eclipse Zenoh", and related names are trademarks of the
Eclipse Foundation or their respective owners. They appear here only
as nominative attribution of origin. This SDK is not an official
Eclipse or Zenoh product and is not endorsed by the Eclipse Foundation.
