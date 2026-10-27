# RnR 実行環境セットアップ

## 1. 目的

この文書では、ビルド済みの RnR を実行専用 Windows PC で使用するための
セットアップ手順を記録する。

Visual Studio および OpenHaptics SDK をインストールしない、
できるだけ小さい実行環境を構築することを目的とする。

## 2. 動作確認済み構成

以下の構成で、Touch を使用した RnR の Record / Replay 動作を確認した。

- Windows 11
- RnR Release / x64
- Touch Device Driver 2023.10.11
- Touch
- Visual Studio：インストール不要
- OpenHaptics SDK 3.5.0：インストール不要

## 3. Touch Device Driver のインストール

Touch Device Driver 2023.10.11 をインストールする。

インストール後、Touch Smart Setup を起動し、Touch を接続する。

以下が正常に表示されることを確認する。

- Touch デバイス
- Default Device
- COM ポート

## 4. RnR.exe の配置

開発PCで Release / x64 構成をビルドして生成した `RnR.exe` を、
実行PCの任意のフォルダへコピーする。

例：

```text
C:\RnR\RnR.exe
```

Debug版ではなく、Release / x64版を使用する。

## 5. hd.dll の配置

Touch Device Driver 2023.10.11 のインストール環境では、
以下に x64版 `hd.dll` が存在することを確認した。

```text
C:\Program Files\3D Systems\Touch Device Drivers\OpenHaptics_Demos\HapticDemo\HapticDemo_Data\Plugins\x86_64\hd.dll
```

この `hd.dll` を `RnR.exe` と同じフォルダへコピーする。

例：

```text
C:\RnR\
├── RnR.exe
└── hd.dll
```

今回の検証では、OpenHaptics SDK 3.5.0 を実行PCへインストールしなくても、
この構成で RnR を実行できた。

## 6. RnR の起動

Touch を接続し、Touch Smart Setup で正常に認識されていることを確認する。

コマンドプロンプトから、例えば以下のように起動する。

```cmd
cd /d C:\RnR
RnR.exe
```

Touch が認識され、キャリブレーション後に RnR のメニューが表示されることを確認する。

## 7. 動作確認

以下について実機で動作確認済み。

- Touch の認識
- RnR の起動
- キャリブレーション
- Record
- Replay

## 8. Smart App Control

検証に使用したPCでは、`hd.dll` を `RnR.exe` と同じフォルダへ配置した際、
Windows の Smart App Control により実行がブロックされた。

Smart App Control をオフにした後、RnR は正常に起動し、
Record / Replay まで動作した。

Smart App Control の設定変更は Windows のセキュリティ設定に影響するため、
PCの管理方針を確認した上で行うこと。

なお、Smart App Control は一度オフにすると、
通常の設定操作だけでは再度オンに戻せない場合がある。

## 9. 実行PCで不要だったもの

今回の実機検証では、以下をインストールせずに RnR が動作した。

- Visual Studio
- Visual Studio C++ 開発環境
- OpenHaptics SDK 3.5.0

したがって、これらは RnR のビルドには必要だが、
ビルド済み Release / x64版 RnR の実行には不要であることを確認した。

## 10. 未検証事項

以下はまだ確認していない。

- Visual C++ Redistributable を別途インストールする必要がある条件
- Smart App Control を有効にしたまま RnR を配布・実行する方法
- Touch Device Driver の別バージョンでの動作
- Windows 11 以外での動作
- Win32版 RnR の動作

## 11. 動作確認日

2026-10-27
