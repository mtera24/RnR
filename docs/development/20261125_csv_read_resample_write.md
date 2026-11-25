# RnR：CSV読み込み・リサンプリング・書き出し機能の整備

実施期間：2026年11月24日～25日

## 1. 開発目的

CSV Version 1.0に基づき、運動データと音声データを統一的に取り扱う基盤を整備する。

旧来の固定長配列に依存した処理から、MotionSample構造体とstd::vectorを用いた処理へ段階的に移行する。

## 2. CSV読み込み

- readMotionCSV()への読み込み処理の統一。
- 列順序の変更への対応。
- 省略可能な列への対応。
- 未知のメタデータ・列の読み飛ばし。
- 不正な数値、時刻、列数の検出。
- 10000点を超えるCSVの読み込み確認。
- RawDataReadyとReplayReadyの分離。
- 安全性検査が完成するまで実機Replayを禁止。

## 3. リサンプリング

- std::vectorによるリサンプリング処理の再実装。
- RAWデータとリサンプリング結果の分離。
- 基本サンプリング間隔10ms。
- RAWデータの終了時刻を保持。
- XYZ座標の線形補間。
- ボタン状態の直前値保持。
- 音声周波数・振幅のhold／linear補間。
- 音声・ボタン状態の変化時刻を保持。
- linear補間時の音声データの折れ点を保持。
- ジンバル角度は暫定的に直前値保持。
- 最大生成点数1000000点。
- メモリ確保失敗への対応。
- ResampledDataReadyの導入。

## 4. CSV検査関数の共通化

以下の関数を読み込み・書き出し・リサンプリングで共用する。

- validateCSVMetadata()
- validateCSVSampleTime()
- validateMotionSample()
- validateMotionData()

CSV形式の妥当性検査と実機再生の安全性検査は明確に区別する。

## 5. CSV書き出し

共通関数writeMotionCSV()を実装した。

- CSV Version 1.0形式。
- メタデータの出力。
- 省略可能な列の有無の保持。
- 列順序の標準化。
- double型の復元に必要な有効桁数での出力。
- ロケールに依存しない数値出力。
- 書き込み・クローズ時のエラー検査。
- 書き出し前のデータ検査。

未知のメタデータ・列は保存しない。

## 6. 試験機能

Touchを初期化せずに実行できる試験モードを整備した。

- --test-csv
- --test-resample
- --test-write-csv

書き出し試験では、CSV読み込み、書き出し、再読み込みを行い、以下を比較する。

- 全サンプルの全項目。
- 既知のメタデータ。
- 省略可能な列の有無。

試験用一時ファイルは、通常モードで試験成功時に削除する。詳細表示モードおよび試験失敗時には残す。

## 7. 最終回帰試験

| 試験項目 | 正常系 | 異常系 | 合格数 |
|---|---:|---:|---:|
| CSV読み込み | 4 | 15 | 19 |
| リサンプリング | 8 | 2 | 10 |
| CSV書き出し | 4 | 15 | 19 |
| 合計 | 16 | 32 | 48 |

全48件で期待どおりの終了コードを確認した。

別途、実機で記録したrecord.csvのラウンドトリップ試験にも成功した。

ただし、書き出し試験の異常系は入力CSVの拒否試験であり、書き出し関数単体の異常系試験ではない。

## 8. Gitコミット

- 35ada09：Add CSV v1.0 conformance tests and verbose sample diagnostics
- 582a5b9：Unify CSV loading and disable replay pending safety validation
- 3de6ff3：Implement vector-based motion resampling and validation tests
- 648c730：Add shared CSV writer and round-trip validation

すべてorigin/mainへpush済み。

## 9. 残存課題

- 書き出し関数単体の異常系試験。
- ファイル書き込み失敗時の動作確認。
- 既存CSVの上書き保護。
- リサンプリング結果の保存処理の統合。
- Record保存処理の統合。
- 旧関数・固定長配列の整理。
- 実機再生の安全性検査。
- Replay処理のvectorへの移行。
- Replay時の音声同期。

現段階では共通書き出し関数は試験モードでのみ使用する。

実機Replayは禁止状態を維持する。