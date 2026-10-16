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

## License election

`flexiv_tdk` depends on **Eclipse zenoh-c 1.9.0** and **zenoh-cpp 1.9.0**
from the official Eclipse repositories and statically links them (with
symbols hidden) into `libflexiv_tdk`. zenoh-c in turn statically links
the Zenoh Rust implementation and the other Rust crates listed in
`licenses/zenoh-c-rust-deps.linux.md` and `licenses/zenoh-c-rust-deps.macos.md`.

Upstream offers zenoh-c, zenoh-cpp, the Zenoh Rust implementation and
several of its crates under a **dual license**: Apache License 2.0
**OR** Eclipse Public License 2.0 (SPDX: `EPL-2.0 OR Apache-2.0`).
There is no separate Apache-only source tree; the dual-licensed
official tags are what TDK uses.

**Flexiv Robotics elects Apache License 2.0** for every component in
this SDK that is offered under `EPL-2.0 OR Apache-2.0`.

- Recipients receive that elected code under Apache-2.0, not EPL-2.0.
- Flexiv does not sublicense that elected code under EPL-2.0.
- Where any other embedded component offers a choice of licenses,
  Flexiv elects Apache-2.0 if it is offered. The license elected for
  each Rust crate is shown in the column "License in this SDK" of the
  inventory files.
- `licenses/zenoh.LICENSE` contains the elected Apache-2.0 text.
- `licenses/zenoh-NOTICE.md` is the upstream Zenoh 1.9.0 NOTICE file.
  zenoh-c and zenoh-cpp ship the same notice under their own titles.

## Embedded C/C++ components

| Component                                                       | Version | License in this SDK      | Upstream                                              |
| --------------------------------------------------------------- | ------- | ------------------------ | ----------------------------------------------------- |
| [spdlog](https://github.com/gabime/spdlog)                      | 1.9.2   | MIT                      | https://github.com/gabime/spdlog/tree/v1.9.2          |
| [fmt](https://github.com/fmtlib/fmt) (bundled by spdlog)        | 8.0.1   | MIT                      | https://github.com/fmtlib/fmt/tree/8.0.1              |
| [Eigen](https://gitlab.com/libeigen/eigen)                      | 3.4.0   | MPL-2.0                  | https://gitlab.com/libeigen/eigen/-/tree/3.4.0        |
| [Eclipse zenoh-c](https://github.com/eclipse-zenoh/zenoh-c)     | 1.9.0   | **Apache-2.0 (elected)** | https://github.com/eclipse-zenoh/zenoh-c/tree/1.9.0   |
| [Eclipse zenoh-cpp](https://github.com/eclipse-zenoh/zenoh-cpp) | 1.9.0   | **Apache-2.0 (elected)** | https://github.com/eclipse-zenoh/zenoh-cpp/tree/1.9.0 |

Eigen 3.4.0 is primarily licensed under MPL-2.0, and that license
applies to the Eigen code compiled into `libflexiv_tdk` (including
`Quaternion` and matrix templates). The library is built with
`EIGEN_MPL2_ONLY`, which excludes Eigen's LGPL-licensed code, so
Eigen's `COPYING.GPL` and `COPYING.LGPL` are not shipped. A few Eigen
headers carry other permissive licenses, for example
`Eigen/src/Core/arch/Default/BFloat16.h` (Apache-2.0, Copyright 2017
The TensorFlow Authors).

Copyright notices retained as required by those licenses:

- spdlog: Copyright (c) 2016 Gabi Melman
- fmt: Copyright (c) 2012 - present, Victor Zverovich
- Eigen: Copyright the Eigen authors
- Zenoh: Copyright (c) 2017, 2022 ZettaScale Technology; Copyright (c) Eclipse Zenoh project contributors

## Embedded Rust components

The complete list of Rust crates linked into `libflexiv_tdk`, with
version, upstream license, elected license, copyright notices and full
license texts, is in:

- `licenses/zenoh-c-rust-deps.linux.md` (Linux x86_64 and aarch64)
- `licenses/zenoh-c-rust-deps.macos.md` (macOS arm64)

The Rust standard library is also statically linked. Its copyright and
license notices are in `licenses/rust-std.COPYRIGHT-library.html`.

## MPL-2.0 source code

The MPL-2.0 code in `libflexiv_tdk` is available in Source Code Form at:

- Eigen 3.4.0: https://gitlab.com/libeigen/eigen/-/tree/3.4.0
- option-ext 0.2.0 (Rust crate): https://crates.io/crates/option-ext/0.2.0

## Full license texts

- `licenses/spdlog.LICENSE`
- `licenses/fmt.LICENSE`
- `licenses/eigen.MPL-2.0`
- `licenses/zenoh.LICENSE` (Apache-2.0 as elected by Flexiv)
- `licenses/zenoh-NOTICE.md`
- `licenses/zenoh-c-rust-deps.linux.md`
- `licenses/zenoh-c-rust-deps.macos.md`
- `licenses/rust-std.COPYRIGHT-library.html`


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
