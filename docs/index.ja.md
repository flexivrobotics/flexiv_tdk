# Flexiv TDK ドキュメント

**Flexiv TDK（遠隔操作開発キット）** のドキュメントサイトへようこそ。このサイトでは、TDK のセットアップと統合に役立つユーザーマニュアル、API リファレンス、よくある質問を提供しています。

## Flexiv TDK とは？

Flexiv TDK は、Flexiv のアダプティブロボットを使用してカスタムのロボット対ロボットまたはデバイス対ロボットの遠隔操作アプリケーションを構築するための SDK です。**高忠実度知覚フィードバック**による同期された力誘導運動を可能にし、**LAN**（ローカルエリアネットワーク）と **WAN**（インターネット）の両方の接続をサポートします。

TDK は、エンボディド AI データ収集、放射線医学、危険な化学実験などの分野で幅広いアプリケーションシナリオを誇ります。

🎬 **[Flexiv's TDK | Teleoperation Made Simple](https://www.youtube.com/watch?v=H0e9FSZIa14)**
*（下の画像をクリックして再生）*
<p align="center">
  <a href="https://www.youtube.com/watch?v=H0e9FSZIa14" target="_blank">
    <img src="https://img.youtube.com/vi/H0e9FSZIa14/hqdefault.jpg" alt="TDK Demo 1" width="350" style="margin-right:10px;" />
  </a>
  <a href="https://www.youtube.com/watch?v=udkddqxth5Q" target="_blank">
    <img src="https://img.youtube.com/vi/udkddqxth5Q/hqdefault.jpg" alt="TDK Demo 2" width="350" />
  </a>
</p>

## 接触型マニピュレーションベンチマーク

Flexiv TDK は [Manipulation Net ペグインホールリーダーボード](https://manipulation-net.org/leaderboards/peg_in_hole.html)に掲載されています。[Manipulation Net](https://manipulation-net.org)は、実世界でのロボットマニピュレーションのための公開ベンチマークであり、あらゆるロボットで、いつでもどこでも、大規模に実施できます。これは、コンプライアントな挿入、アライメント、力覚遠隔操作ワークフローを含む TDK のユースケースに関連する接触型マニピュレーション能力の外部証拠となります。

## 主な特徴

- **高忠実度知覚フィードバック**：100% の触覚フィードバック透過性により、人間の操作の忠実度を確保します。
- **より優れた物理的ヒューマンロボットインタラクション**：リーダーロボットはいつでも位置や向きを変更できます。再結合時には相対運動のみがマッピングされ、絶対姿勢の制約はありません。
- **選択的デカルト拘束**：特定の方向に沿って運動を拘束し、より速く、より正確なタスク実行を実現します。
- **堅牢な力/モーメント保護**：ロボットとワークピースの損傷を防ぎ、接触時の本質的安全性を確保します。
- **LAN/WAN サポート**：LAN と WAN の両方に対応する柔軟なネットワーク構成。

## クイックリンク

- [ユーザーマニュアル](user-manual/overview.md)
- [API リファレンス](api/doxygen/index.html)
- [よくある質問](qa/index.md)
- [GitHub リポジトリ](https://github.com/flexivrobotics/flexiv_tdk)

## ドキュメント構成

- **ユーザーマニュアル**：セットアップ、インストール、例、操作ガイダンス。
- **API リファレンス**：Doxygen 生成の C++ API ドキュメント。
- **よくある質問**：よくある質問とトラブルシューティングのヒント。

> 注：API リファレンスは Doxygen で生成され、このサイトとともに GitHub Pages で公開されています。
