# RnR 開発環境セットアップ

## 1. 目的

この文書では、RnR のソースコードを新しい Windows PC でビルド・開発するための
開発環境セットアップ手順を記録する。

目的は、現在動作している開発環境を別PCでも再現できるようにすることである。

実行専用PCのセットアップについては `SETUP_RUNTIME.md` を参照すること。

---

## 2. 動作確認済み開発環境

2026-10-27 時点で、以下の環境で RnR のビルドおよび Touch を使用した実行を確認している。

### OS

- Windows 11

### Git

- Git for Windows 2.47.0.windows.1

このバージョンへの固定は必須ではない。
上記は動作確認済み環境の記録である。

### Visual Studio

- Microsoft Visual Studio Community 2022
- Version 17.14.41
- C++によるデスクトップ開発

### MSVC Toolset

以下の個別コンポーネントを追加している。

- MSVC v142 - VS 2019 C++ x64/x86 ビルドツール
- v14.29-16.11

RnR のプロジェクトファイル `RnR.vcxproj` は、
Platform Toolset として `v142` を指定している。

### Windows SDK

動作確認環境では以下を使用している。

- Windows 11 SDK 10.0.26100.7705

`RnR.vcxproj` では Windows SDK のバージョンとして `10.0` が指定されており、
10.0.26100.7705 への固定はしていない。

### OpenHaptics

- OpenHaptics SDK 3.5.0

インストール先：

```text
C:\OpenHaptics\Developer\3.5.0
```

環境変数：

```text
OH_SDK_BASE=C:\OpenHaptics\Developer\3.5.0
```

### Touch Device Driver

- Touch Device Driver 2023.10.11

---

## 3. Git のインストール

Git for Windows をインストールする。

動作確認環境：

```text
git version 2.47.0.windows.1
```

インストール後、Git Bash などで以下を実行し、Git が使用できることを確認する。

```bash
git --version
```

---

## 4. Visual Studio 2022 のインストール

Visual Studio Community 2022 をインストールする。

Visual Studio Installer の「ワークロード」で、以下を選択する。

```text
C++によるデスクトップ開発
```

動作確認環境では Visual Studio Community 2022 Version 17.14.41 を使用している。

Visual Studio のバージョンを 17.14.41 に固定する必要はないが、
新しいバージョンを使用した場合はビルド・動作を確認すること。

---

## 5. MSVC v142 Toolset の追加

RnR は `v142` Toolset を使用する。

Visual Studio Installer の「個別のコンポーネント」で `v142` を検索し、
以下を追加する。

```text
MSVC v142 - VS 2019 C++ x64/x86 ビルドツール
```

動作確認環境：

```text
v14.29-16.11
```

`RnR.vcxproj` では以下の設定が使用されている。

```xml
<PlatformToolset>v142</PlatformToolset>
```

Visual Studio 2022 の標準 Toolset へ自動的にリターゲットせず、
当面は v142 を維持する。

---

## 6. Windows SDK のインストール

Visual Studio Installer から Windows SDK をインストールする。

動作確認環境：

```text
Windows 11 SDK 10.0.26100.7705
```

`RnR.vcxproj` では以下の設定が使用されている。

```xml
<WindowsTargetPlatformVersion>10.0</WindowsTargetPlatformVersion>
```

したがって、現在のプロジェクトは特定の 10.0.x SDK バージョンには固定されていない。

---

## 7. OpenHaptics SDK 3.5.0 のインストール

OpenHaptics SDK 3.5.0 をインストールする。

動作確認環境でのインストール先：

```text
C:\OpenHaptics\Developer\3.5.0
```

環境変数 `OH_SDK_BASE` が以下を指していることを確認する。

コマンドプロンプト：

```cmd
echo %OH_SDK_BASE%
```

期待される例：

```text
C:\OpenHaptics\Developer\3.5.0
```

---

## 8. OpenHaptics SDK の確認

ヘッダファイルが存在することを確認する。

```cmd
dir "%OH_SDK_BASE%\include\HD\hd.h"
```

動作確認環境では以下に存在する。

```text
C:\OpenHaptics\Developer\3.5.0\include\HD\hd.h
```

x64 Debug 用ライブラリについて、以下の存在も確認する。

```cmd
dir "%OH_SDK_BASE%\lib\x64\Debug\hd.lib"
dir "%OH_SDK_BASE%\utilities\lib\x64\Debug\hdu.lib"
```

動作確認済みのファイル：

```text
C:\OpenHaptics\Developer\3.5.0\lib\x64\Debug\hd.lib
C:\OpenHaptics\Developer\3.5.0\utilities\lib\x64\Debug\hdu.lib
```

---

## 9. RnR プロジェクトの OpenHaptics 設定

`RnR.vcxproj` では、OpenHaptics のヘッダおよびライブラリを
`OH_SDK_BASE` を基準として参照している。

### Include directories

```text
$(OH_SDK_BASE)\include
$(OH_SDK_BASE)\utilities\include
```

### Link libraries

```text
hd.lib
hdu.lib
```

### Library directories

```text
$(OH_SDK_BASE)\lib\$(Platform)\$(Configuration)
$(OH_SDK_BASE)\utilities\lib\$(Platform)\$(Configuration)
$(OH_SDK_BASE)\lib\$(Platform)
```

このため、OpenHaptics SDK のインストール先をソースコードや
プロジェクトファイルへ直接記述する必要はない。

---

## 10. Touch Device Driver のインストール

Touch Device Driver 2023.10.11 をインストールする。

インストール後、Touch Smart Setup を起動し、Touch を接続する。

以下を確認する。

- Touch デバイスが認識される
- Default Device として設定される
- COM ポートが表示される
- スタイラスの動きが認識される

動作確認環境では、Touch Device Driver 2023.10.11 で正常動作を確認している。

---

## 11. GitHub から RnR を取得

Git Bash などで、作業用ディレクトリへ移動する。

例：

```bash
cd /c/dev/RnR
```

GitHub から RnR リポジトリを clone する。

```bash
git clone https://github.com/mtera24/RnR.git
```

clone 後、リポジトリへ移動する。

```bash
cd RnR
```

現在の主要ファイルは以下である。

```text
RnR.sln
RnR.vcxproj
RnR.cpp
README.md
SETUP_RUNTIME.md
SETUP_DEVELOPMENT.md
```

---

## 12. Visual Studio でプロジェクトを開く

`RnR.sln` を Visual Studio 2022 で開く。

Visual Studio から Toolset や SDK のリターゲットを提案された場合、
現在の動作確認済み環境を維持するため、当面は自動変更しない。

特に Platform Toolset は `v142` を維持する。

---

## 13. Debug / x64 のビルド

Visual Studio の構成を以下に設定する。

```text
Configuration: Debug
Platform:      x64
```

ビルドを実行する。

動作確認環境では、以下が生成される。

```text
x64\Debug\RnR.exe
```

Debug / x64 について、ビルドおよび Touch 実機での起動を確認済み。

---

## 14. Release / x64 のビルド

Visual Studio の構成を以下に設定する。

```text
Configuration: Release
Platform:      x64
```

ビルドを実行する。

動作確認環境では、以下が生成される。

```text
x64\Release\RnR.exe
```

Release / x64 についても、ビルドおよび Touch 実機での動作を確認済み。

実行専用PCへ配布する場合は、Debug版ではなく Release / x64版を使用する。

---

## 15. Release ビルド時の Post-build Event

現在の `RnR.vcxproj` には、Release 系構成に Post-build Event が設定されている。

内容：

```cmd
echo on
mkdir $(SolutionDir)\..\..\bin\$(PlatformName)\HD
copy "$(IntDir)\$(TargetFileName)" "$(SolutionDir)\..\..\bin\$(PlatformName)\HD
```

このため、Release ビルド時には `RnR.exe` が別の `bin` ディレクトリにもコピーされる。

この Post-build Event は既存プロジェクトから引き継がれたものであり、
現在の RnR に必要かどうかは未確認である。

必要性を確認するまでは削除しない。

---

## 16. 現在確認されている警告

現在のソースでは、ビルド時にいくつかの警告が表示される。

確認されているもの：

- C4101：未使用のローカル変数
- C4244：`double` から `int` への変換
- C4018：signed / unsigned の比較
- C4715：すべての制御パスで値を返していない可能性
- LNK4099：`HDU.pdb` が見つからない

Debug 構成では、古い `/Gm` オプションに関する D9035 も確認されている。

これらは現時点では既存ソース由来の警告として残しており、
環境構築時には変更していない。

Debug / x64 および Release / x64 のビルドと実機動作は、
これらの警告が存在する状態で確認済みである。

---

## 17. 動作確認

開発環境構築後、Touch を接続して RnR を起動する。

以下を確認する。

- Touch が認識される
- キャリブレーションできる
- RnR のメニューが表示される
- Record が動作する
- Replay が動作する

Release / x64 については、これらの実機動作を確認済み。

---

## 18. 実行専用PCとの違い

開発PCでは以下が必要である。

- Git
- Visual Studio 2022
- C++によるデスクトップ開発
- MSVC v142 Toolset
- Windows SDK
- OpenHaptics SDK 3.5.0
- Touch Device Driver

一方、実行専用PCでは Visual Studio と OpenHaptics SDK を
インストールせずに RnR が動作することを確認済みである。

実行専用PCについては `SETUP_RUNTIME.md` を参照すること。

---

## 19. 未検証事項

以下はまだ確認していない。

- この文書だけを使用した完全な新規開発PCでの再構築
- Win32 構成のビルドおよび動作
- DebugAcademicEdition 構成
- ReleaseAcademicEdition 構成
- v143 Toolset への移行
- OpenHaptics SDK 3.5.0 以外でのビルド
- Touch Device Driver 2023.10.11 以外での動作
- Windows 11 以外での開発
- Release の Post-build Event が現在も必要かどうか

これらは必要になった時点で検証する。

---

## 20. 動作確認日

2026-10-27
