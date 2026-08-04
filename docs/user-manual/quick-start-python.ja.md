# クイックスタート（Python）

このガイドでは、Flexiv TDK Python パッケージのインストールと実行について説明します。パッケージは [PyPI](https://pypi.org/project/flexivtdk/) で公開されています。

## 1) パッケージのインストール

```bash
python3.x -m pip install spdlog flexivtdk
```

> `3.x` はお使いの Python バージョン（例：3.10）に置き換えてください。

## 2) インストールの確認

```bash
python3.x
>>> import flexivtdk
>>> flexivtdk.__version__
```

## 3) このリポジトリの Python サンプルを実行する

一般ユーザーが `sudo` なしで高優先度（リアルタイム）スレッドを作成できるように、システムにリアルタイムおよび nice 優先度の制限を設定します（一度だけ設定すればOKです）：

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

ログアウトして再ログイン（または再起動）すると設定が有効になります。その後、すべてのサンプルを `sudo` なしで実行できます。

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

使用方法の詳細は各サンプルのソースコードを確認してください。

!!! warning "重要"

    人とロボットのインタラクションは TCP の力/モーメントから読み取られます。そのため、片手でロボットのエンドエフェクタを、もう片方の手でロボットの別のリンクを握ることは避けてください。これはよくある操作ミスです。


## ヒント

- ロボットのネットワーク接続が安定していることを確認してください。
- WAN 遠隔操作については[時刻同期 (WAN)](time-sync.md)を参照してください。
- リアルタイム性能については[リアルタイムカーネル](real-time-kernel.md)を参照してください。
