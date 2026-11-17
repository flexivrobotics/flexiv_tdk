# よくある質問

## Q: どのプラットフォームがサポートされていますか？
**A:** Ubuntu 22.04+（x86_64 および aarch64）は C++ と Python 3.10/3.12/3.14 をサポートします。macOS 14+（arm64）は C++ と Python 3.10/3.12 をサポートします。ツールチェーンは GCC ≥ 9.4（Linux）または Apple Clang ≥ 15（macOS）、CMake ≥ 3.16.3 です。

## Q: リアルタイムカーネルは必要ですか？
**A:** 必須ではありませんが、低遅延または RT カーネルは遠隔操作の応答性と安定性を向上させます。[リアルタイムカーネル](../user-manual/real-time-kernel.md)を参照してください。

## Q: WAN 遠隔操作の時刻同期はどうすればよいですか？
**A:** Chrony を使用して両端のシステムクロックを同期します。[時刻同期 (WAN)](../user-manual/time-sync.md)を参照してください。

## Q: API リファレンスはどこにありますか？
**A:** Doxygen API リファレンスは GitHub Pages の `api/doxygen/index.html` に公開されています。[API リファレンス](../../api/doxygen/index.html)を参照してください。

## Q: サポートはどこで受けられますか？
**A:** 営業担当者にご連絡いただくか、GitHub で issue を作成するか、https://www.flexiv.com/contact から Flexiv までお問い合わせください。
