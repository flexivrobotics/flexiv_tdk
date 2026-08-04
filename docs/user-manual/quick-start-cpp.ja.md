# クイックスタート（C++）

このガイドでは、Flexiv TDK を CMake パッケージとしてビルドして使用する方法を説明します。

## 1) ビルド依存関係のインストール

```bash
sudo apt install build-essential cmake cmake-qt-gui -y
```

## 2) インストールディレクトリの選択

例：

```bash
mkdir -p ~/tdk_install
```

## 3) サードパーティ依存関係のビルドとインストール

```bash
cd flexiv_tdk/thirdparty
bash build_and_install_dependencies.sh ~/tdk_install
```

> GitHub へのインターネットアクセスが必要です。

## 4) TDK の構成とインストール

```bash
cd flexiv_tdk
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install
cmake --build . --target install --config Release
```

## 5) プロジェクトで TDK をリンクする

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
```

## 6) サンプルの実行

```bash
cd flexiv_tdk/example/build
sudo ./<program_name> [arguments]
```

一般ユーザーが `sudo` なしで高優先度（リアルタイム）スレッドを作成できるように、システムにリアルタイムおよび nice 優先度の制限を設定します（一度だけ設定すればOKです）：

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

ログアウトして再ログイン（または再起動）すると設定が有効になります。その後、すべてのサンプルを `sudo` なしで実行できます。

詳細は [API リファレンス](../api/doxygen/index.html)を参照してください。
