// Guide and Record Version 2.2
// 2023.07.10
// by M.Teranishi
// Change log :
// flexible recording time limit. 
/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:

  CommandJointTorque.cpp
  

Description: 

  This example demonstrates commanding joint torques to the Phantom device. 
*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include <vector>
#include <cstdint>
#include <new>

// CSVファイルの読み込み・文字列解析
#include <fstream>
#include <string>
#include <sstream>

#include <cmath>
#include <stdexcept>

#include <set>

#include <algorithm>

#include <iomanip>
#include <limits>
#include <locale>

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
# include <mmsystem.h>  // Windows音声再生API
#pragma comment(lib, "winmm.lib")
#else
#include <time.h>
# include "conio.h"
# include <string.h>
#define FALSE 0
#define TRUE 1
#endif
#include <direct.h>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

// 運動・姿勢・操作状態・音出力情報を
// 共通の時刻で管理するための1サンプル分のデータ構造
struct MotionSample
{
    uint64_t time_ms = 0;
    double position[3] = {};
    double gimbalAngles[3] = {};
    uint32_t buttons = 0;
    double audio_freq_hz = 0.0;
    double audio_amplitude = 0.0;
};

// Recordによって取得した実測データ
static std::vector<MotionSample> recordedData;

// CSVから読み込んだ未加工の運動データ
static std::vector<MotionSample> rawData;

// 安全性検査および必要なリサンプリングを経て
// Replayに使用する運動データ
static std::vector<MotionSample> replayData;


static bool ReplayMode = false;
static bool ReplayReady = false; // whether replay data loaded or not.

static const char* Language = "en";
static bool VerboseMode = false;

/* # of record */

// 記録点数の暫定上限（メモリ使用量の制限）
static constexpr size_t MAX_RECORD_SAMPLES = 10000;

/* interval for recording [ms]*/
#define INTV_REC 10 
// filename for record
#define FNAME_REC "record.csv" // up dir due to debug exec.
// filename for playback
#define FNAME_REPLAY "replay.csv" 

#define FNAME_REPLAY_RESAMPLE "replay_resampled.csv"

// line buffer length for csv reading.
#define LEN_LINE_CSV 1024
// Maximum length of replay data (raw)
#define MAX_LEN_REPLAY_RAW 20000
// リサンプリングの基本時間間隔 [ms]
#define INTV_RESAMPLE_REPLAY_MS 10

// リサンプリングによる生成点数の暫定上限。
// 過大なメモリ消費を防止するための制限であり、
// 実機再生の安全性を保証するものではない。
static constexpr size_t MAX_RESAMPLED_SAMPLES = 1000000;

// 旧固定長配列用の上限。
// 旧再生処理の廃止時に削除する。
#define MAX_LEN_REPLAY_RESAMPLE 10000

#define BUFSIZE MAX_PATH


static int end_idx_resample;

static DWORD time_start;

static hduVector3Dd replay_position_raw[MAX_LEN_REPLAY_RAW];
static DWORD replay_timeidx_raw[MAX_LEN_REPLAY_RAW];

static hduVector3Dd replay_position_resample[MAX_LEN_REPLAY_RESAMPLE];
static DWORD replay_timeidx_resample[MAX_LEN_REPLAY_RESAMPLE];

static hduVector3Dd wellPos;

HDCallbackCode HDCALLBACK guideAndRecordCallback(void *data);
HDSchedulerHandle hGravityWell = HD_INVALID_HANDLE;

void mainLoop(void);
bool initDemo(void);

#include <cmath>
#include <cstdint>
#include <vector>

// 440Hzの正弦波を1秒分生成する。
// 後でwaveOutによる連続再生に使用する。
std::vector<int16_t> CreateTone()
{
    const int sampleRate = 44100;
    const double frequency = 440.0;
    const double amplitude = 3000.0;
    const double pi = 3.14159265358979323846;

    std::vector<int16_t> samples(sampleRate);

    for (int i = 0; i < sampleRate; ++i) {
        double t = static_cast<double>(i) / sampleRate;

        samples[i] = static_cast<int16_t>(
            amplitude * std::sin(2.0 * pi * frequency * t)
            );
    }

    return samples;
}

// 音声再生デバイスのハンドル
HWAVEOUT hWaveOut = nullptr;

// 音声デバイスを初期化する
bool InitAudio()
{
    WAVEFORMATEX format = {};

    format.wFormatTag = WAVE_FORMAT_PCM;
    format.nChannels = 1;
    format.nSamplesPerSec = 44100;
    format.wBitsPerSample = 16;
    format.nBlockAlign = 2;
    format.nAvgBytesPerSec = 44100 * 2;

    MMRESULT result = waveOutOpen(
        &hWaveOut,
        WAVE_MAPPER,
        &format,
        0,
        0,
        CALLBACK_NULL
    );

    return result == MMSYSERR_NOERROR;
}

// 二重バッファ方式で使用する音声データ
// 2つのバッファを交互に再生し、音切れを防ぐ

const int AUDIO_BUFFER_COUNT = 2;

// 各バッファの音声データ
std::vector<int16_t> toneSamples[AUDIO_BUFFER_COUNT];

// 各バッファの再生管理情報
WAVEHDR waveHeader[AUDIO_BUFFER_COUNT] = {};


// 2つの音声バッファを準備する
bool PrepareAudio()
{
    if (hWaveOut == nullptr) {
        return false;
    }

    // 2つのバッファを順番に初期化する
    for (int i = 0; i < AUDIO_BUFFER_COUNT; ++i) {

        // 440Hzの正弦波を生成
        toneSamples[i] = CreateTone();

        // バッファ管理情報を初期化
        waveHeader[i] = {};

        // 音声データの場所とサイズを設定
        waveHeader[i].lpData =
            reinterpret_cast<LPSTR>(toneSamples[i].data());

        waveHeader[i].dwBufferLength =
            static_cast<DWORD>(
                toneSamples[i].size() * sizeof(int16_t)
                );

        // Windowsの音声デバイスに登録
        MMRESULT result = waveOutPrepareHeader(
            hWaveOut,
            &waveHeader[i],
            sizeof(WAVEHDR)
        );

        if (result != MMSYSERR_NOERROR) {
            return false;
        }
    }

    return true;
}

// 音声デバイスと2つのバッファを安全に解放する
void CloseAudio()
{
    // 音声デバイスが開かれていなければ何もしない
    if (hWaveOut == nullptr) {
        return;
    }

    // 再生中の音声を停止する
    waveOutReset(hWaveOut);

    // 2つのバッファをそれぞれ解放する
    for (int i = 0; i < AUDIO_BUFFER_COUNT; ++i) {

        // 登録済みのバッファだけを解放する
        if (waveHeader[i].dwFlags & WHDR_PREPARED) {
            waveOutUnprepareHeader(
                hWaveOut,
                &waveHeader[i],
                sizeof(WAVEHDR)
            );
        }
    }

    // 音声デバイスを閉じる
    waveOutClose(hWaveOut);

    hWaveOut = nullptr;
}


// 2つの音声バッファを再生キューに登録する
bool StartAudio()
{
    if (hWaveOut == nullptr) {
        return false;
    }

    // 2つのバッファを順番に登録する
    for (int i = 0; i < AUDIO_BUFFER_COUNT; ++i) {

        // バッファが準備されているか確認する
        if (!(waveHeader[i].dwFlags & WHDR_PREPARED)) {
            return false;
        }

        // すでに再生キューにあれば登録しない
        if (waveHeader[i].dwFlags & WHDR_INQUEUE) {
            continue;
        }

        // 音声バッファを再生キューへ登録する
        MMRESULT result = waveOutWrite(
            hWaveOut,
            &waveHeader[i],
            sizeof(WAVEHDR)
        );

        if (result != MMSYSERR_NOERROR) {
            return false;
        }
    }

    return true;
}

// 音声再生を停止する
void StopAudio()
{
    // 音声デバイスが初期化されていなければ何もしない
    if (hWaveOut == nullptr) {
        return;
    }

    // 再生中の音声を停止し、キューを空にする
    waveOutReset(hWaveOut);
}

// 再生が完了した音声バッファがあるか確認する
bool IsAudioFinished()
{
    if (hWaveOut == nullptr) {
        return false;
    }

    // 2つのバッファを順番に確認する
    for (int i = 0; i < AUDIO_BUFFER_COUNT; ++i) {

        // 再生完了したバッファを検出
        if (waveHeader[i].dwFlags & WHDR_DONE) {
            return true;
        }
    }

    return false;
}

void PrintCwd()
{
    char* buffer;
    // Get the current working directory:
    if ((buffer = _getcwd(NULL, 0)) == NULL)
        perror("_getcwd error");
    else
    {
        printf("CWD : %s \nLength: %zu\n", buffer, strlen(buffer));
        free(buffer);
    }

    return;

}

void PrintHelp()
{
    static const char help_en[] =

        "GuideAndRecord Help\n"
        "---\n"
        "Touch stylus controls\n"
        "Button 1: Start recording.\n"
        "Press Button 1 again during recording to stop.\n"
        "\n"
        "Keyboard controls\n"
        "R: Start recording.\n"
        "Press any key during recording to stop.\n"
        "L: Load replay data.\n"
        "G: Start guide mode.\n"
        "D: Load recorded data.\n"
        "P: Print device state.\n"
        "C: Continuously print device state.\n"
        "A: Load and resample RAW replay data.\n"
        "W: Write resampled replay data.\n"
        "H: Show help.\n"
        "Q: Quit the program.\n"
        "---";

    static const char help_ja[] =
        "GuideAndRecord ヘルプ\n"
        "---\n"
        "Touchスタイラス操作\n"
        "ボタン1：記録を開始します。\n"
        "記録中にボタン1を再度押すと終了します。\n"
        "\n"
        "キーボード操作\n"
        "R：記録を開始します。\n"
        "記録中に何かキーを押すと終了します。\n"
        "L：再生データを読み込みます。\n"
        "G：ガイドモードを開始します。\n"
        "D：記録データを読み込みます。\n"
        "P：デバイスの状態を表示します。\n"
        "C：デバイスの状態を連続表示します。\n"
        "A：RAW再生データを読み込み、リサンプリングします。\n"
        "W：リサンプリングデータを書き出します。\n"
        "H：ヘルプを表示します。\n"
        "Q：プログラムを終了します。\n"
        "---";

    if (strcmp(Language, "ja") == 0) {
        printf("\n%s\n", help_ja);
    }
    else {
        printf("\n%s\n", help_en);
    }

    // 作業ディレクトリはCSVの保存先確認などに使う診断情報。
    // 通常モードではヘルプ表示時の不要な読み上げを抑える。
    if (VerboseMode) {
        PrintCwd();
    }
}

/* Synchronization structure. */
typedef struct
{
    HDdouble position[3];
    HDdouble gimbalAngles[3];
    HDint buttons;
} DeviceStateStruct;

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData)
{
    DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;

    hdGetDoublev(HD_CURRENT_POSITION, pState->position);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, pState->gimbalAngles);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &pState->buttons);

    return HD_CALLBACK_DONE;
}

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
void PrintDeviceState(HDboolean bContinuous)
{
    int i;
    DeviceStateStruct state;

    memset(&state, 0, sizeof(DeviceStateStruct));

    do
    {
        hdScheduleSynchronous(GetDeviceStateCallback, &state,
            HD_DEFAULT_SCHEDULER_PRIORITY);

        printf("\n");

        printf("Current position (mm):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.position[i]);
        }
        printf("\n");

        printf("Current Gimbal Angles (rad):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.gimbalAngles[i]);
        }
        printf("\n");
        
        printf("Buttons: %d\n", state.buttons);

        if (bContinuous)
        {
#if defined(WIN32)
       		Sleep(500);
#elif defined(linux)
		struct timespec timeOut;
                timeOut.tv_sec = 0;
                timeOut.tv_nsec = 5*100000000;
                nanosleep(&timeOut, NULL);
#endif

        }

    } while (!_kbhit() && bContinuous);
}

/*****************************************************************************
 Callback that record state.
*****************************************************************************/
void RecordDeviceState()
{

    // 前回の記録データを消去する
    recordedData.clear();

    // 記録開始前に必要なメモリを確保する。
    // 確保に失敗した場合は記録を開始しない。
    try
    {
        recordedData.reserve(MAX_RECORD_SAMPLES);
    }
    catch (const std::bad_alloc&)
    {
        if (strcmp(Language, "ja") == 0)
        {
            fprintf(stderr,
                "メモリ確保に失敗しました。記録を中止します。\n");
        }
        else
        {
            fprintf(stderr,
                "Memory allocation failed. Recording cancelled.\n");
        }

        return;
    }

    // ボタン1が一度離されたかを記録する。
    // 開始時に押していたボタンで、すぐ記録終了しないようにする。
    bool button1Released = false;

    // ボタン2の直前の押下状態
    bool previousButton2Pressed = false;

    // Touchの現在の状態を取得するための変数
    DeviceStateStruct buttonState = {};

    /* # of record */
    /* interval for recording [ms]*/
    // #define INTV_REC 10
   // 記録可能時間の算出
    double lim_rec_time =
        MAX_RECORD_SAMPLES * INTV_REC / 1000.0;

    int i;
    double end_time;
    DWORD now, time_next;

    FILE* fp_rec_csv = NULL;

    DeviceStateStruct state;

    memset(&state, 0, sizeof(DeviceStateStruct));
    
    // Record開始前の案内
// --lang ja の場合は日本語、それ以外は英語で表示する。
    if (strcmp(Language, "ja") == 0) {
        fprintf(stderr, "記録モードを開始します。\n");
        fprintf(stderr, "記録可能時間は最大%.0f秒です。\n", lim_rec_time);
        fprintf(stderr, "記録中はTouchのボタン1、またはキーボードのキーで終了できます。\n");
        fprintf(stderr, "3秒後に記録を開始します。\n");
    }
    else {
        fprintf(stderr, "Recording mode.\n");
        fprintf(stderr, "Maximum recording time: %.0f seconds.\n", lim_rec_time);
        fprintf(stderr, "Press Touch Button 1 or any keyboard key to stop recording.\n");
        fprintf(stderr, "Recording starts in 3 seconds.\n");
    }

    // 従来の3秒カウントダウンは維持する。
    // NVDAとの同期問題は、後の段階で別途改善する。
    for (int i = 3; i > 0; --i) {
        fprintf(stderr, "%d\n", i);
        Sleep(1000);
    }

    if (strcmp(Language, "ja") == 0) {
        fprintf(stderr, "記録を開始しました。\n");
    }
    else {
        fprintf(stderr, "Recording started.\n");
    }
    
    now = time_start = timeGetTime();

    hdScheduleSynchronous(GetDeviceStateCallback, &state,
        HD_DEFAULT_SCHEDULER_PRIORITY);

    time_next = now;

    // 記録開始時点の運動・操作状態を格納する
    MotionSample sample;

    sample.time_ms = 0;

    memcpy(sample.position, state.position,
        sizeof(sample.position));

    memcpy(sample.gimbalAngles, state.gimbalAngles,
        sizeof(sample.gimbalAngles));

    sample.buttons = static_cast<uint32_t>(state.buttons);

    // ボタン2の状態から音出力指示を生成する
    if ((state.buttons & HD_DEVICE_BUTTON_2) != 0)
    {
        sample.audio_freq_hz = 440.0;
        sample.audio_amplitude = 3000.0 / 32767.0;
    }

    // 最初のサンプルを動的配列に追加する
    recordedData.push_back(sample);

    do
    {
        time_next += INTV_REC;
        while (now < time_next) { //wait until next time
            now = timeGetTime();
        }
        hdScheduleSynchronous(GetDeviceStateCallback, &state,
            HD_DEFAULT_SCHEDULER_PRIORITY);

        // 運動・操作状態・音出力情報を同一サンプルとして管理する
        MotionSample sample;

        // 記録開始からの相対時刻 [ms]
        sample.time_ms = static_cast<uint64_t>(now - time_start);

        memcpy(sample.position, state.position,
            sizeof(sample.position));

        memcpy(sample.gimbalAngles, state.gimbalAngles,
            sizeof(sample.gimbalAngles));

        sample.buttons = static_cast<uint32_t>(state.buttons);

        // ボタン2の状態に対応する音出力指示を保存する
        if ((state.buttons & HD_DEVICE_BUTTON_2) != 0)
        {
            sample.audio_freq_hz = 440.0;
            sample.audio_amplitude = 3000.0 / 32767.0;
        }

        recordedData.push_back(sample);

        // Touchの現在のボタン状態を取得する
        hdScheduleSynchronous(
            GetDeviceStateCallback,
            &buttonState,
            HD_DEFAULT_SCHEDULER_PRIORITY
        );

        // ボタン2の現在の押下状態
        bool button2Pressed =
            (buttonState.buttons & HD_DEVICE_BUTTON_2) != 0;

        // ボタン2が押された瞬間に音声再生を開始する
        if (button2Pressed && !previousButton2Pressed) {
            if (!StartAudio()) {
                fprintf(stderr, "音声再生の開始に失敗しました。\n");
            }
        }

        // ボタン2が離された瞬間に音声を停止する
        if (!button2Pressed && previousButton2Pressed) {
            StopAudio();
        }

        // ボタン2を押している間、再生済みバッファを補充する
        if (button2Pressed && IsAudioFinished()) {
            if (!StartAudio()) {
                fprintf(stderr, "音声の連続再生に失敗しました。\n");
            }
        }

        // 次回の判定に備えて状態を保存する
        previousButton2Pressed = button2Pressed;

        // ボタン1が押されているか調べる
        bool button1Pressed =
            (buttonState.buttons & HD_DEVICE_BUTTON_1) != 0;

        // ボタン1が離されたことを確認する
        if (!button1Pressed) {
            button1Released = true;
        }

        // 一度離された後、再び押されたら記録を終了する
        if (button1Released && button1Pressed) {
            break;
        }

        // キー入力または記録点数の上限到達で記録を終了する
    } while (!_kbhit() &&
        recordedData.size() < MAX_RECORD_SAMPLES);

    // Record終了時は、ボタン2の状態にかかわらず音声を停止する
    StopAudio();

    fprintf(stderr, "record end.\n");

    // write to file
    fp_rec_csv = fopen(FNAME_REC, "w");

    if (fp_rec_csv == NULL) {
        fprintf(stderr, "file open failed! : %s\n", FNAME_REC);
        exit(1);
    }
    else {
        // RnR CSV形式の識別情報
        fprintf(fp_rec_csv, "# format=RnR\n");
        fprintf(fp_rec_csv, "# version=1.0\n");
        fprintf(fp_rec_csv, "# source=recorded\n");

        // CSV列名（物理量の単位を含む）
        fprintf(fp_rec_csv,
            "time_ms,x_mm,y_mm,z_mm,"
            "gimbal1_rad,gimbal2_rad,gimbal3_rad,"
            "buttons,audio_freq_hz,audio_amplitude\n");

        // 動的配列に格納された全サンプルを保存する
        for (const MotionSample& s : recordedData)
        {
            fprintf(fp_rec_csv,
                "%llu,%.17g,%.17g,%.17g,"
                "%.17g,%.17g,%.17g,"
                "%u,%.17g,%.17g\n",
                static_cast<unsigned long long>(s.time_ms),
                s.position[0],
                s.position[1],
                s.position[2],
                s.gimbalAngles[0],
                s.gimbalAngles[1],
                s.gimbalAngles[2],
                static_cast<unsigned int>(s.buttons),
                s.audio_freq_hz,
                s.audio_amplitude);
        }

        fclose(fp_rec_csv);

    }

    // 記録開始から最終サンプルまでの経過時間 [秒]
    // 空のvectorに対するback()の呼び出しを防止する
    double recordedSeconds = 0.0;

    if (!recordedData.empty())
    {
        recordedSeconds = recordedData.back().time_ms / 1000.0;
    }

    // NVDAが読み上げやすいように記録結果の要約を表示する
    if (strcmp(Language, "ja") == 0) {
        fprintf(stderr,
            "記録終了。記録時間 %.1f 秒、記録点数 %zu 点。%s に保存しました。\n",
            recordedSeconds, recordedData.size(), FNAME_REC);
    }
    else {
        fprintf(stderr,
            "Recording finished. Duration %.1f seconds, %zu samples. Saved to %s.\n",
            recordedSeconds, recordedData.size(), FNAME_REC);
    }
}

/*****************************************************************************
 Callback that replay state.
*****************************************************************************/
void ReplayDeviceState()
{
    // 新しいCSV読み込み処理への移行期間中は、
    // 安全性検査が未実装のため実機再生を禁止する。
    constexpr bool ENABLE_REPLAY = false;

    if (!ENABLE_REPLAY)
    {
        if (strcmp(Language, "ja") == 0)
        {
            fprintf(stderr,
                "安全性検査が未実装のため、"
                "現在は再生できません。\n");
        }
        else
        {
            fprintf(stderr,
                "Replay is disabled until "
                "safety validation is implemented.\n");
        }

        return;
    }

    DWORD now, time_replay_start, time_next;

    int i;
    int i_resample;

    if (!ReplayReady) { // replay data not loaded successfully
        if (strcmp(Language, "ja") == 0) {
            fprintf(stderr, "再生データが読み込まれていません。\n");
            fprintf(stderr, "Lコマンドで再生データを読み込んでください。\n");
        }
        else {
            fprintf(stderr, "replay data not loaded successfully.\n");
            fprintf(stderr, "Please confirm replay data loaded by L command.\n");
        }
        return;
    }

    // guide to initial gravity well position
    if (strcmp(Language, "ja") == 0) {
        fprintf(stderr, "再生モードを開始します。\n");
        fprintf(stderr, "スタイラスをキャリブレーション位置から外し、手に持ってください。\n");
        fprintf(stderr, "これからスタイラスを開始位置まで案内します。\n");
        fprintf(stderr, "準備ができたら、何かキーを押してください。\n");
    }
    else {
        fprintf(stderr, "Replay mode :\n");
        fprintf(stderr, "   Please remove stylus from calibration well, and hold any position.\n");
        fprintf(stderr, "   When you got ready, press any key.");
    }

    fflush(stdin);
    getch();

    if (strcmp(Language, "ja") == 0) {
    }
    else {
        fprintf(stderr, "Now I guide you to the starting position.\n");
    }

    wellPos[0] = replay_position_resample[0][0];
    wellPos[1] = replay_position_resample[0][1];
    wellPos[2] = replay_position_resample[0][2];
    ReplayMode = true;

    if (strcmp(Language, "ja") == 0) {
        fprintf(stderr, "スタイラスが開始位置まで移動したら、何かキーを押してください。\n");
    }
    else {
        fprintf(stderr, "When your stylus moved to start position, press any key.\n");
    }

    fflush(stdin);
    getch();

    if (strcmp(Language, "ja") == 0) {
        printf("3秒後に再生を開始します。\n");
    }
    else {
        printf("Replay starts after 3 sec.count.");
    }

    for (i = 3; i > 0; --i) {
        printf("%d   ", i);
        Sleep(1000);
    }

    if (strcmp(Language, "ja") == 0) {
        printf("開始します。\n");
    }
    else {
        printf("...go!\n");
    }

    // replay
    now = time_replay_start = timeGetTime();

    for (i_resample = 1; i_resample < (end_idx_resample - 1); ++i_resample) {
        time_next = time_replay_start + replay_timeidx_resample[i_resample];

        while (now < time_next) { // wait until next time
            now = timeGetTime();
        }

        // move gravity well
        memcpy(wellPos, replay_position_resample[i_resample],
            sizeof(hduVector3Dd));
    }

    if (strcmp(Language, "ja") == 0) {
        fprintf(stderr, "再生が終了しました。何かキーを押してください。\n");
    }
    else {
        fprintf(stderr, "press any key to quit replay mode.\n");
    }

    fflush(stdin);
    getch();

    ReplayMode = false;

    if (strcmp(Language, "ja") == 0) {
        fprintf(stderr, "再生モードを終了しました。\n");
    }
    else {
        fprintf(stderr, "replay mode disabled.\n");
    }

    return;
}


// 共通CSV読み込み関数の前方宣言
static bool readMotionCSV(const char* filename);

// 指定されたCSVファイルを読み込み、rawDataに格納する。
// CSV形式の検査結果を戻り値として返す。
// この段階では実機再生を許可しない。
bool read_resampled_ReplayData(const char* filename)
{
    return readMotionCSV(filename);
}

// 文字列の前後にある空白・タブ・改行文字を除去する。
// 文字列内部の空白は変更しない。
static std::string trim(const std::string& str)
{
    const std::string whitespace = " \t\r\n";

    size_t first = str.find_first_not_of(whitespace);

    // 空文字列、または空白文字だけの場合
    if (first == std::string::npos)
    {
        return "";
    }

    size_t last = str.find_last_not_of(whitespace);

    return str.substr(first, last - first + 1);
}

// CSVの1行をカンマで分割する。
// 空欄も1つの列として保持する。
// 引用符を含むCSVの解析には対応しない。
static std::vector<std::string> splitCSVLine(
    const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;

    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }

    // 行末がカンマの場合、最後の空欄も保持する。
    if (!line.empty() && line.back() == ',')
    {
        fields.push_back("");
    }

    return fields;
}

// 指定された列名に対応する列番号を返す。
// 列番号は0から始まる。
// 見つからない場合は-1を返す。
static int findCSVColumn(
    const std::vector<std::string>& columns,
    const std::string& name)
{
    for (size_t i = 0; i < columns.size(); ++i)
    {
        if (trim(columns[i]) == name)
        {
            return static_cast<int>(i);
        }
    }

    return -1;
}

// CSVの文字列を実数値に変換する。
// 不正な文字列、変換可能範囲外の値、
// NaN、無限大の場合はfalseを返す。
static bool parseCSVDouble(
    const std::string& field,
    double& value)
{
    std::string str = trim(field);

    if (str.empty())
    {
        return false;
    }

    try
    {
        size_t pos = 0;
        double result = std::stod(str, &pos);

        // 文字列全体が数値として解釈されたか確認する。
        if (pos != str.size())
        {
            return false;
        }

        // NaNおよび無限大を拒否する。
        if (!std::isfinite(result))
        {
            return false;
        }

        value = result;
        return true;
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
    catch (const std::out_of_range&)
    {
        return false;
    }
}

// CSVの文字列を64ビット符号なし整数に変換する。
// 負数、小数、空欄、範囲外の値は拒否する。
static bool parseCSVUInt64(
    const std::string& field,
    uint64_t& value)
{
    std::string str = trim(field);

    if (str.empty())
    {
        return false;
    }

    // 10進数字以外の文字を拒否する。
    for (char c : str)
    {
        if (c < '0' || c > '9')
        {
            return false;
        }
    }

    try
    {
        size_t pos = 0;
        uint64_t result = std::stoull(str, &pos, 10);

        if (pos != str.size())
        {
            return false;
        }

        value = result;
        return true;
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
    catch (const std::out_of_range&)
    {
        return false;
    }
}

// CSVヘッダにおける各項目の列番号を管理する。
// -1は、その列が存在しないことを示す。
struct MotionCSVColumns
{
    // 必須列
    int time_ms = -1;
    int x_mm = -1;
    int y_mm = -1;
    int z_mm = -1;

    // 省略可能な列
    int gimbal1_rad = -1;
    int gimbal2_rad = -1;
    int gimbal3_rad = -1;
    int buttons = -1;
    int audio_freq_hz = -1;
    int audio_amplitude = -1;
};

// RnR CSVファイルのメタデータを管理する。
struct MotionCSVMetadata
{
    // 必須メタデータ
    std::string format;
    std::string version;

    // 省略可能なメタデータ
    std::string title;
    std::string device;
    std::string source;
    std::string created_at;

    // 公称サンプリング間隔 [ms]
    // 空文字列は未指定を示す。
    std::string nominal_interval_ms;

    // 音出力情報の補間方式
    // 未指定の場合は直前の値を保持する。
    std::string interpolation_audio_freq = "hold";
    std::string interpolation_audio_amplitude = "hold";
};

// 読み込んだCSVファイルのメタデータ
static MotionCSVMetadata rawMetadata;

// 読み込んだCSVファイルの列構成
// 省略された項目と値がゼロの項目を区別するために保持する。
static MotionCSVColumns rawColumns;

// CSVファイルの読み込みと形式検査が
// 正常に完了したことを示す。
static bool RawDataReady = false;

// リサンプリングが正常に完了し、
// replayDataが生成されたことを示す。
// 実機再生の安全性を保証するものではない。
static bool ResampledDataReady = false;

// メタデータ行をキーと値に分解する。
// 書式は "# key=value" とする。
// キーと値の前後にある空白は除去する。
static bool splitCSVMetadata(
    const std::string& line,
    std::string& key,
    std::string& value)
{
    std::string str = trim(line);

    // メタデータ行の先頭は '#' でなければならない。
    if (str.empty() || str[0] != '#')
    {
        return false;
    }

    // キーと値の区切り位置を取得する。
    size_t pos = str.find('=');

    if (pos == std::string::npos)
    {
        return false;
    }

    // '#' を除いた部分からキーを取得する。
    key = trim(str.substr(1, pos - 1));

    // 最初の '=' より後ろを値として取得する。
    value = trim(str.substr(pos + 1));

    // 空のキーは許容しない。
    return !key.empty();
}

// メタデータのキーと値を構造体に格納する。
// 同じキーが複数回出現した場合は拒否する。
// 未知のキーは将来の仕様拡張に備えて無視する。
static bool setCSVMetadata(
    const std::string& key,
    const std::string& value,
    MotionCSVMetadata& metadata,
    std::set<std::string>& seenKeys)
{
    // キーの重複を検査する。
    if (!seenKeys.insert(key).second)
    {
        return false;
    }

    if (key == "format")
        metadata.format = value;
    else if (key == "version")
        metadata.version = value;
    else if (key == "title")
        metadata.title = value;
    else if (key == "device")
        metadata.device = value;
    else if (key == "source")
        metadata.source = value;
    else if (key == "created_at")
        metadata.created_at = value;
    else if (key == "nominal_interval_ms")
        metadata.nominal_interval_ms = value;
    else if (key == "interpolation_audio_freq")
        metadata.interpolation_audio_freq = value;
    else if (key == "interpolation_audio_amplitude")
        metadata.interpolation_audio_amplitude = value;

    return true;
}

// メタデータがRnR CSV Version 1.0の仕様に
// 適合しているか検査する。
static bool validateCSVMetadata(
    const MotionCSVMetadata& metadata)
{
    // ファイル形式とバージョンを検査する。
    if (metadata.format != "RnR" ||
        metadata.version != "1.0")
    {
        return false;
    }

    // 公称サンプリング間隔 [ms]
    // 指定されている場合は正の数値を要求する。
    if (!metadata.nominal_interval_ms.empty())
    {
        double interval = 0.0;

        if (!parseCSVDouble(
            metadata.nominal_interval_ms, interval))
        {
            return false;
        }

        if (interval <= 0.0)
        {
            return false;
        }
    }

    // 音の周波数の補間方式を検査する。
    if (metadata.interpolation_audio_freq != "hold" &&
        metadata.interpolation_audio_freq != "linear")
    {
        return false;
    }

    // 音の振幅の補間方式を検査する。
    if (metadata.interpolation_audio_amplitude != "hold" &&
        metadata.interpolation_audio_amplitude != "linear")
    {
        return false;
    }

    return true;
}


// CSVヘッダを解析し、各項目の列番号を設定する。
// 列名の重複、空の列名、必須列の欠落を拒否する。
static bool parseCSVHeader(
    const std::vector<std::string>& fields,
    MotionCSVColumns& columns)
{
    // 列名の重複および空欄を検査する。
    for (size_t i = 0; i < fields.size(); ++i)
    {
        std::string name = trim(fields[i]);

        if (name.empty())
        {
            return false;
        }

        for (size_t j = 0; j < i; ++j)
        {
            if (name == trim(fields[j]))
            {
                return false;
            }
        }
    }

    // 必須列の列番号を取得する。
    columns.time_ms = findCSVColumn(fields, "time_ms");
    columns.x_mm = findCSVColumn(fields, "x_mm");
    columns.y_mm = findCSVColumn(fields, "y_mm");
    columns.z_mm = findCSVColumn(fields, "z_mm");

    // 必須列が1つでも欠落していれば読み込みを拒否する。
    if (columns.time_ms < 0 ||
        columns.x_mm < 0 ||
        columns.y_mm < 0 ||
        columns.z_mm < 0)
    {
        return false;
    }

    // 省略可能な列の列番号を取得する。
    columns.gimbal1_rad = findCSVColumn(fields, "gimbal1_rad");
    columns.gimbal2_rad = findCSVColumn(fields, "gimbal2_rad");
    columns.gimbal3_rad = findCSVColumn(fields, "gimbal3_rad");
    columns.buttons = findCSVColumn(fields, "buttons");
    columns.audio_freq_hz = findCSVColumn(fields, "audio_freq_hz");
    columns.audio_amplitude = findCSVColumn(fields, "audio_amplitude");

    return true;
}

// 1サンプル分の運動データがCSV仕様に
// 適合しているか検査する。
// 時刻の検査および実機再生の安全性検査は別途行う。
static bool validateMotionSample(
    const MotionSample& sample)
{
    // XYZ座標 [mm] の有限値検査
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(sample.position[axis]))
        {
            return false;
        }
    }

    // ジンバル角度 [rad] の有限値検査
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(sample.gimbalAngles[axis]))
        {
            return false;
        }
    }

    // ボタン状態（ビットマスク）
    if (sample.buttons > 3)
    {
        return false;
    }

    // 音声周波数 [Hz]
    if (!std::isfinite(sample.audio_freq_hz) ||
        sample.audio_freq_hz < 0.0)
    {
        return false;
    }

    // 音声振幅（無次元、0.0～1.0）
    if (!std::isfinite(sample.audio_amplitude) ||
        sample.audio_amplitude < 0.0 ||
        sample.audio_amplitude > 1.0)
    {
        return false;
    }

    return true;
}

// CSVのデータ行から運動データを生成する。
// 必須項目の欠落や不正な数値を拒否する。
// fieldsにはヘッダと同じ列数が必要である。
static bool parseCSVSample(
    const std::vector<std::string>& fields,
    const MotionCSVColumns& columns,
    size_t expectedColumnCount,
    MotionSample& sample)
{
    // ヘッダとデータ行の列数が一致するか検査する。
    if (fields.size() != expectedColumnCount)
    {
        return false;
    }

    // 全メンバを初期状態に戻す。
    sample = MotionSample{};

    // 時刻 [ms]
    if (!parseCSVUInt64(
        fields[columns.time_ms], sample.time_ms))
    {
        return false;
    }

    // X座標 [mm]
    if (!parseCSVDouble(
        fields[columns.x_mm], sample.position[0]))
    {
        return false;
    }

    // Y座標 [mm]
    if (!parseCSVDouble(
        fields[columns.y_mm], sample.position[1]))
    {
        return false;
    }

    // Z座標 [mm]
    if (!parseCSVDouble(
        fields[columns.z_mm], sample.position[2]))
    {
        return false;
    }

    // ジンバル角度 [rad]
// 対応する列が存在する場合だけ読み込む。
// 列が存在しない場合は初期値を維持する。

    if (columns.gimbal1_rad >= 0)
    {
        if (!parseCSVDouble(
            fields[columns.gimbal1_rad],
            sample.gimbalAngles[0]))
        {
            return false;
        }
    }

    if (columns.gimbal2_rad >= 0)
    {
        if (!parseCSVDouble(
            fields[columns.gimbal2_rad],
            sample.gimbalAngles[1]))
        {
            return false;
        }
    }

    if (columns.gimbal3_rad >= 0)
    {
        if (!parseCSVDouble(
            fields[columns.gimbal3_rad],
            sample.gimbalAngles[2]))
        {
            return false;
        }
    }

    // ボタン状態（ビットマスク）
// ボタン1とボタン2の組み合わせのみ許容する。
    if (columns.buttons >= 0)
    {
        uint64_t buttons = 0;

        if (!parseCSVUInt64(
            fields[columns.buttons], buttons))
        {
            return false;
        }

        // 有効な値は0～3である。
        if (buttons > 3)
        {
            return false;
        }

        sample.buttons = static_cast<uint32_t>(buttons);
    }

    // 音の周波数 [Hz]
// 負の周波数は許容しない。
    if (columns.audio_freq_hz >= 0)
    {
        if (!parseCSVDouble(
            fields[columns.audio_freq_hz],
            sample.audio_freq_hz))
        {
            return false;
        }

    }

    // 音の振幅（0.0～1.0の正規化値）
    if (columns.audio_amplitude >= 0)
    {
        if (!parseCSVDouble(
            fields[columns.audio_amplitude],
            sample.audio_amplitude))
        {
            return false;
        }

    }

    // 数値変換後のサンプルを共通検査関数で検査する。
    return validateMotionSample(sample);
}

// 新しいサンプルの時刻がCSV仕様に適合するか検査する。
// 最初の時刻は0msとし、以降は厳密な単調増加を要求する。
// サンプルの時刻 [ms] がCSV仕様に適合するか検査する。
// previousがnullptrの場合は先頭サンプルとして扱う。
static bool validateCSVSampleTime(
    const MotionSample* previous,
    const MotionSample& sample)
{
    // 最初のサンプルは時刻0msでなければならない。
    if (previous == nullptr)
    {
        return sample.time_ms == 0;
    }

    // 直前のサンプルより時刻が進んでいることを確認する。
    return sample.time_ms > previous->time_ms;
}

// RnR CSV Version 1.0形式のファイルを読み込む。
// CSV形式の妥当性を検査する。
// 実機再生の安全性検査は別の関数で行う。
static bool readMotionCSV(const char* filename)
try
{
    // 古いデータによる誤再生を防止する。
    ReplayReady = false;
    ResampledDataReady = false;
    RawDataReady = false;

    // 前回の読み込み結果を初期化する。
    rawData.clear();
    replayData.clear();

    rawMetadata = MotionCSVMetadata{};
    rawColumns = MotionCSVColumns{};

    // CSVファイルを開く。
    std::ifstream file(filename);

    if (!file.is_open())
    {
        fprintf(stderr,
            "Cannot open file: %s\n", filename);

        return false;
    }

    std::string line;
    std::set<std::string> seenKeys;
    size_t lineNumber = 0;
    bool headerFound = false;

    // メタデータを読み込み、最初のヘッダ行を取得する。
    while (std::getline(file, line))
    {
        ++lineNumber;
        line = trim(line);

        // 空行は読み飛ばす。
        if (line.empty())
        {
            continue;
        }

        // '#' で始まる行をメタデータとして解析する。
        if (line[0] == '#')
        {
            std::string key;
            std::string value;

            if (!splitCSVMetadata(line, key, value) ||
                !setCSVMetadata(
                    key, value, rawMetadata, seenKeys))
            {
                fprintf(stderr,
                    "Invalid metadata at line %zu.\n",
                    lineNumber);

                return false;
            }
        }
        else
        {
            // 最初の非メタデータ行をヘッダ行とする。
            headerFound = true;
            break;
        }
    }

    // ヘッダ行が存在することを確認する。
    if (!headerFound)
    {
        fprintf(stderr, "CSV header not found.\n");
        return false;
    }

    // メタデータがCSV仕様に適合するか検査する。
    if (!validateCSVMetadata(rawMetadata))
    {
        fprintf(stderr, "Invalid CSV metadata.\n");
        return false;
    }

    // ヘッダ行をカンマで分割する。
    std::vector<std::string> headerFields =
        splitCSVLine(line);

    // 列名を解析し、各項目の列番号を取得する。
    // 必須列の欠落や列名の重複があれば読み込みを中止する。
    if (!parseCSVHeader(headerFields, rawColumns))
    {
        fprintf(stderr,
            "Invalid CSV header at line %zu.\n",
            lineNumber);

        return false;
    }

    // データ行の列数検査に使用する。
    const size_t expectedColumnCount =
        headerFields.size();

    // CSVのデータ行を1行ずつ読み込む。
    while (std::getline(file, line))
    {
        ++lineNumber;
        line = trim(line);

        // 空行は読み飛ばす。
        if (line.empty())
        {
            continue;
        }

        // データ行をカンマで分割する。
        std::vector<std::string> fields =
            splitCSVLine(line);

        MotionSample sample;

        // 各項目の数値および値域を検査する。
        if (!parseCSVSample(
            fields, rawColumns,
            expectedColumnCount, sample))
        {
            fprintf(stderr,
                "Invalid CSV data at line %zu.\n",
                lineNumber);

            rawData.clear();
            return false;
        }

        // 時刻の単調増加を検査する。
        if (!validateCSVSampleTime(
            rawData.empty() ? nullptr : &rawData.back(),
            sample))
        {
            fprintf(stderr,
                "Invalid timestamp at line %zu.\n",
                lineNumber);

            rawData.clear();
            return false;
        }

        // 検査に合格したサンプルを格納する。
        rawData.push_back(sample);
    }

    // ファイルの読み込みが正常に終了したか検査する。
    // EOFへの到達は正常終了として扱う。
    if (file.bad() || !file.eof())
    {
        fprintf(stderr, "CSV file read error.\n");

        rawData.clear();
        return false;
    }

    // 運動データが1点も存在しない場合は拒否する。
    if (rawData.empty())
    {
        fprintf(stderr, "CSV contains no motion data.\n");

        return false;
    }

    // CSVの読み込みと形式検査が正常に完了した。
    RawDataReady = true;

    // 実機再生の安全性検査は未実施であるため、
    // 再生許可は与えない。
    ReplayReady = false;

    // 読み込み結果を利用者に通知する。
    if (strcmp(Language, "ja") == 0)
    {
        fprintf(stderr,
            "CSVデータを読み込みました。%zu点。\n",
            rawData.size());
    }
    else
    {
        fprintf(stderr,
            "CSV data loaded. %zu samples.\n",
            rawData.size());
    }

    // ファイル名はVerboseモードでのみ表示する。
    if (VerboseMode)
    {
        fprintf(stderr,
            "Source file: %s\n", filename);
    }

    return true;
}

catch (const std::bad_alloc&)
{
    // メモリ確保失敗時は読み込み結果を破棄し、
    // 実機再生を禁止する。
    RawDataReady = false;
    ReplayReady = false;

    rawData.clear();
    replayData.clear();

    if (strcmp(Language, "ja") == 0)
    {
        fprintf(stderr,
            "メモリ確保に失敗しました。"
            "CSV読み込みを中止します。\n");
    }
    else
    {
        fprintf(stderr,
            "Memory allocation failed. "
            "CSV loading cancelled.\n");
    }

    return false;
}

// 運動データ全体がCSV仕様に適合するか検査する。
// 実機再生の安全性検査は含まない。
static bool validateMotionData(
    const std::vector<MotionSample>& data)
{
    // 空のデータは許容しない。
    if (data.empty())
    {
        fprintf(stderr,
            "No motion data.\n");

        return false;
    }

    const MotionSample* previous = nullptr;

    for (size_t i = 0; i < data.size(); ++i)
    {
        const MotionSample& sample = data[i];

        // 時刻 [ms] の検査
        if (!validateCSVSampleTime(previous, sample))
        {
            fprintf(stderr,
                "Invalid timestamp at sample %zu.\n",
                i + 1);

            return false;
        }

        // サンプル内の数値検査
        if (!validateMotionSample(sample))
        {
            fprintf(stderr,
                "Invalid motion data at sample %zu.\n",
                i + 1);

            return false;
        }

        previous = &sample;
    }

    return true;
}

// 指定されたCSVファイルの読み込みを検証する。
// Touchの運動制御は行わない。
static void testReadMotionCSV(
    const char* filename = FNAME_REC)
{
    bool result = readMotionCSV(filename);

    fprintf(stderr, "\n--- CSV Load Test ---\n");

    fprintf(stderr, "Result: %s\n",
        result ? "true" : "false");

    fprintf(stderr, "RawDataReady: %s\n",
        RawDataReady ? "true" : "false");

    fprintf(stderr, "ReplayReady: %s\n",
        ReplayReady ? "true" : "false");

    fprintf(stderr, "Samples: %zu\n",
        rawData.size());

    if (result && !rawData.empty())
    {
        fprintf(stderr, "First time: %llu ms\n",
            static_cast<unsigned long long>(
                rawData.front().time_ms));

        fprintf(stderr, "Last time: %llu ms\n",
            static_cast<unsigned long long>(
                rawData.back().time_ms));

        // 詳細表示モードでは、最初のサンプルの
        // 各メンバが正しく読み込まれたか確認する。
        if (VerboseMode)
        {
            const MotionSample& s = rawData.front();

            fprintf(stderr, "\n--- First Sample ---\n");

            // 位置 [mm]
            fprintf(stderr, "X: %.17g\n", s.position[0]);
            fprintf(stderr, "Y: %.17g\n", s.position[1]);
            fprintf(stderr, "Z: %.17g\n", s.position[2]);

            // ジンバル角度 [rad]
            fprintf(stderr, "Gimbal1: %.17g\n",
                s.gimbalAngles[0]);
            fprintf(stderr, "Gimbal2: %.17g\n",
                s.gimbalAngles[1]);
            fprintf(stderr, "Gimbal3: %.17g\n",
                s.gimbalAngles[2]);

            // ボタン状態（ビットマスク）
            fprintf(stderr, "Buttons: %u\n",
                static_cast<unsigned int>(s.buttons));

            // 音の周波数 [Hz] と正規化振幅
            fprintf(stderr, "Audio frequency: %.17g Hz\n",
                s.audio_freq_hz);
            fprintf(stderr, "Audio amplitude: %.17g\n",
                s.audio_amplitude);

            // 任意列の存在有無を表示する。
            // 列番号が-1の場合、その列は存在しない。
            fprintf(stderr, "\n--- Optional Columns ---\n");

            fprintf(stderr, "Gimbal1: %s\n",
                rawColumns.gimbal1_rad >= 0 ? "present" : "absent");

            fprintf(stderr, "Gimbal2: %s\n",
                rawColumns.gimbal2_rad >= 0 ? "present" : "absent");

            fprintf(stderr, "Gimbal3: %s\n",
                rawColumns.gimbal3_rad >= 0 ? "present" : "absent");

            fprintf(stderr, "Buttons: %s\n",
                rawColumns.buttons >= 0 ? "present" : "absent");

            fprintf(stderr, "Audio frequency: %s\n",
                rawColumns.audio_freq_hz >= 0 ? "present" : "absent");

            fprintf(stderr, "Audio amplitude: %s\n",
                rawColumns.audio_amplitude >= 0 ? "present" : "absent");

        }
    }

    fprintf(stderr, "--- End of Test ---\n");
}


// Recordで保存したCSVファイルを読み込む。
// CSV形式の検査を行い、rawDataに格納する。
// この段階では実機再生を許可しない。
void read_RecordData()
{
    readMotionCSV(FNAME_REC);
}


// RAW運動データをCSVファイルから読み込む。
// 時刻の単位はCSV Version 1.0に従い[ms]とする。
// 読み込んだデータはrawDataに格納する。
// この段階では実機再生を許可しない。
void read_raw_ReplayData()
{
    readMotionCSV(FNAME_REPLAY);
}



// rawDataからリサンプリング済みの
// replayDataを生成する。
// 実機再生の安全性検査は別の関数で行う。
bool resampleReplayData()
{
    // 前回の処理結果と再生許可を無効化する。
    ReplayReady = false;
    ResampledDataReady = false;
    replayData.clear();

    // CSV読み込みと形式検査が完了しているか確認する。
    if (!RawDataReady || rawData.empty())
    {
        fprintf(stderr,
            "Valid RAW data is not available.\n");

        return false;
    }

    // リサンプリングの基本時間間隔 [ms]
    const uint64_t interval_ms = INTV_RESAMPLE_REPLAY_MS;

    // 元データの最終時刻 [ms]
    const uint64_t endTime_ms = rawData.back().time_ms;

    // 基本時間間隔で生成される区間数
    const uint64_t intervals = endTime_ms / interval_ms;

    // 点数計算時の整数オーバーフローと
    // 過大なメモリ消費を防止する。
    if (intervals >= MAX_RESAMPLED_SAMPLES)
    {
        fprintf(stderr,
            "Resampling sample limit exceeded.\n");

        return false;
    }

    // 開始時刻0msのサンプルを含めた点数
    uint64_t sampleCount = intervals + 1;

    // 最終時刻が基本時間間隔の整数倍でない場合、
    // 元データの終点を追加する。
    if (endTime_ms % interval_ms != 0)
    {
        ++sampleCount;
    }

    // 終点の追加後も生成点数の上限を検査する。
    if (sampleCount > MAX_RESAMPLED_SAMPLES)
    {
        fprintf(stderr,
            "Resampling sample limit exceeded.\n");

        return false;
    }

    // リサンプリング対象となる時刻 [ms] の一覧。
    // 基本サンプリング時刻に加えて、
    // 音声・ボタンの変化時刻も格納する。
    std::vector<uint64_t> targetTimes_ms;

    // 基本生成点数に必要なメモリを事前に確保する。
    // 音声・ボタンの変化時刻に対応する追加点は、
    // 後続の処理で扱う。
    try
    {
        targetTimes_ms.reserve(
            static_cast<size_t>(sampleCount));

        replayData.reserve(
            static_cast<size_t>(sampleCount));
    }
    catch (const std::bad_alloc&)
    {
        fprintf(stderr,
            "Memory allocation failed. "
            "Resampling cancelled.\n");

        return false;
    }

    // 基本サンプリング時刻 [ms] の一覧を生成する。
    for (uint64_t i = 0; i < sampleCount; ++i)
    {
        if (i == sampleCount - 1)
        {
            targetTimes_ms.push_back(endTime_ms);
        }
        else
        {
            targetTimes_ms.push_back(i * interval_ms);
        }
    }

    // 音声・ボタンの変化時刻 [ms] を追加する。
    try
    {
        for (size_t i = 1; i < rawData.size(); ++i)
        {
            const MotionSample& prev = rawData[i - 1];
            const MotionSample& curr = rawData[i];

            // ボタン状態の変化を検出する。
            const bool buttonsChanged =
                curr.buttons != prev.buttons;

            // 音声周波数の変化を検出する。
            const bool freqChanged =
                curr.audio_freq_hz != prev.audio_freq_hz;

            // 音声振幅の変化を検出する。
            const bool amplitudeChanged =
                curr.audio_amplitude != prev.audio_amplitude;

            // 音声に線形補間が指定されているか確認する。
            const bool audioLinear =
                rawMetadata.interpolation_audio_freq == "linear" ||
                rawMetadata.interpolation_audio_amplitude == "linear";

            // 線形補間の場合は、補間区間の境界を保存するため、
            // 値の変化にかかわらずRAWデータの時刻を追加する。
            if (buttonsChanged ||
                freqChanged ||
                amplitudeChanged ||
                audioLinear)
            {
                // 基本サンプリング時刻と重複する場合は、
                // 追加する必要がない。
                if (curr.time_ms % interval_ms == 0 ||
                    curr.time_ms == endTime_ms)
                {
                    continue;
                }

                // 追加時刻を含めた生成点数の上限検査。
                // 過大なメモリ消費を防止する。
                if (targetTimes_ms.size() >=
                    MAX_RESAMPLED_SAMPLES)
                {
                    fprintf(stderr,
                        "Resampling sample limit exceeded.\n");

                    return false;
                }

                targetTimes_ms.push_back(curr.time_ms);
            }
        }
    }
    catch (const std::bad_alloc&)
    {
        fprintf(stderr,
            "Memory allocation failed. "
            "Resampling cancelled.\n");

        return false;
    }


    // サンプリング時刻 [ms] を昇順に並べ替える。
    std::sort(
        targetTimes_ms.begin(),
        targetTimes_ms.end());

    // 重複するサンプリング時刻を削除する。
    targetTimes_ms.erase(
        std::unique(
            targetTimes_ms.begin(),
            targetTimes_ms.end()),
        targetTimes_ms.end());

    // 補間に使用するRAWデータの左側のインデックス。
    size_t leftIndex = 0;


    // 登録された各時刻について補間処理を行う。
    for (const uint64_t targetTime_ms : targetTimes_ms)
    {

        // 目標時刻以下で最も新しいRAWデータを探索する。
        // 配列の範囲外を参照しないようにする。
        while (leftIndex + 1 < rawData.size() &&
            rawData[leftIndex + 1].time_ms <= targetTime_ms)
        {
            ++leftIndex;
        }

        // 補間に使用する右側のインデックス。
        size_t rightIndex = leftIndex;

        // 次のRAWデータが存在する場合は、
        // その点を補間の右側とする。
        if (leftIndex + 1 < rawData.size())
        {
            rightIndex = leftIndex + 1;
        }

        // 補間に使用する左右のRAWデータを参照する。
        const MotionSample& left = rawData[leftIndex];
        const MotionSample& right = rawData[rightIndex];

        // 線形補間係数（無次元）
        double alpha = 0.0;

        // 左右の時刻が異なる場合だけ補間係数を計算する。
        // 最終点などでのゼロ除算を防止する。
        if (right.time_ms > left.time_ms)
        {
            alpha =
                static_cast<double>(
                    targetTime_ms - left.time_ms)
                /
                static_cast<double>(
                    right.time_ms - left.time_ms);
        }

        // リサンプリング後の1点分のデータを生成する。
        MotionSample sample{};

        // サンプリング時刻 [ms]
        sample.time_ms = targetTime_ms;

        // XYZ座標 [mm] を線形補間する。
        for (int axis = 0; axis < 3; ++axis)
        {
            sample.position[axis] =
                (1.0 - alpha) * left.position[axis]
                + alpha * right.position[axis];

        }

        // ボタン状態は直前のRAWデータの値を保持する。
        // ボタン状態は離散値なので線形補間しない。
        sample.buttons = left.buttons;

        // 音声周波数 [Hz] の補間
        if (rawMetadata.interpolation_audio_freq == "linear")
        {
            sample.audio_freq_hz =
                (1.0 - alpha) * left.audio_freq_hz
                + alpha * right.audio_freq_hz;
        }
        else
        {
            // 直前値保持
            sample.audio_freq_hz = left.audio_freq_hz;
        }

        // 音声振幅（無次元、0.0～1.0）の補間
        if (rawMetadata.interpolation_audio_amplitude == "linear")
        {
            sample.audio_amplitude =
                (1.0 - alpha) * left.audio_amplitude
                + alpha * right.audio_amplitude;
        }
        else
        {
            // 直前値保持
            sample.audio_amplitude = left.audio_amplitude;
        }


        // ジンバル角度 [rad] は暫定的に直前値を保持する。
        // 角度の周期性を考慮した補間方式は今後検討する。
        // 現在は実機の再生制御には使用しない。
        for (int axis = 0; axis < 3; ++axis)
        {
            sample.gimbalAngles[axis] =
                left.gimbalAngles[axis];
        }

        // 生成されたサンプルがCSV仕様に適合するか検査する。
        if (!validateMotionSample(sample))
        {
            replayData.clear();

            fprintf(stderr,
                "Invalid resampled motion sample.\n");

            return false;
        }

        // 生成点数が上限を超えないことを確認する。
        if (replayData.size() >= MAX_RESAMPLED_SAMPLES)
        {
            replayData.clear();

            fprintf(stderr,
                "Resampling sample limit exceeded.\n");

            return false;
        }

        // 生成したサンプルを末尾に追加する。
        try
        {
            replayData.push_back(sample);
        }
        catch (const std::bad_alloc&)
        {
            // メモリ確保に失敗した場合は、
            // 生成途中のデータをすべて破棄する。
            replayData.clear();

            fprintf(stderr,
                "Memory allocation failed. "
                "Resampling cancelled.\n");

            return false;
        }
    }

    // 追加時刻を含む目標時刻一覧の点数と、
// 実際の生成点数が一致するか確認する。
    if (replayData.size() != targetTimes_ms.size())
    {
        replayData.clear();

        fprintf(stderr,
            "Resampled sample count mismatch.\n");

        return false;
    }

    // RAWデータと終了時刻 [ms] が一致することを確認する。
    if (replayData.empty() ||
        replayData.back().time_ms != endTime_ms)
    {
        replayData.clear();

        fprintf(stderr,
            "Invalid resampled end time.\n");

        return false;
    }

    // 生成された運動データ全体の妥当性を検査する。
    if (!validateMotionData(replayData))
    {
        replayData.clear();

        return false;
    }


    // リサンプリングの正常完了を記録する。
    ResampledDataReady = true;

    // 安全性検査は未実施のため、
    // 実機再生は許可しない。
    ReplayReady = false;

    // 生成結果を利用者に通知する。
    if (strcmp(Language, "ja") == 0)
    {
        fprintf(stderr,
            "リサンプリングが完了しました。%zu点。\n",
            replayData.size());
    }
    else
    {
        fprintf(stderr,
            "Resampling completed. %zu samples.\n",
            replayData.size());
    }

    return true;
}

// Touchを使用せずにCSV読み込みと
// リサンプリングを検証する。
static bool testResampleCSV(const char* filename)
{
    // CSV読み込みと形式検査を行う。
    const bool readResult = readMotionCSV(filename);

    // CSV読み込みが成功した場合のみ、
    // リサンプリングを実行する。
    const bool result =
        readResult && resampleReplayData();


    // 処理結果と各状態フラグを表示する。
    fprintf(stderr, "Result: %s\n",
        result ? "PASS" : "FAIL");

    fprintf(stderr, "RawDataReady: %s\n",
        RawDataReady ? "true" : "false");

    fprintf(stderr, "ResampledDataReady: %s\n",
        ResampledDataReady ? "true" : "false");

    fprintf(stderr, "ReplayReady: %s\n",
        ReplayReady ? "true" : "false");

    // 入力点数と生成点数を表示する。
    fprintf(stderr, "RAW samples: %zu\n",
        rawData.size());

    fprintf(stderr, "Resampled samples: %zu\n",
        replayData.size());

    // 詳細表示モードでは生成された各サンプルを表示する。
    // 大量出力を防ぐため、100点以下の場合に限定する。
    if (result && VerboseMode &&
        replayData.size() <= 100)
    {
        fprintf(stderr,
            "time_ms,x_mm,y_mm,z_mm,"
            "gimbal1_rad,gimbal2_rad,gimbal3_rad,"
            "buttons,audio_freq_hz,audio_amplitude\n");

        for (const MotionSample& sample : replayData)
        {
            fprintf(stderr,
                "%llu,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%u,%.6f,%.6f\n",
                static_cast<unsigned long long>(
                    sample.time_ms),
                sample.position[0],
                sample.position[1],
                sample.position[2],
                sample.gimbalAngles[0],
                sample.gimbalAngles[1],
                sample.gimbalAngles[2],
                static_cast<unsigned int>(sample.buttons),
                sample.audio_freq_hz,
                sample.audio_amplitude);
        }
    }

    return result;
}

// RnR CSV Version 1.0形式で運動データを書き出す。
// 正常終了時はtrue、失敗時はfalseを返す。
// 実機再生の許可状態は変更しない。
static bool writeMotionCSV(
    const char* filename,
    const std::vector<MotionSample>& data,
    const MotionCSVMetadata& metadata,
    const MotionCSVColumns& columns)
{


    // メタデータがCSV Version 1.0の仕様に
    // 適合しているか検査する。
    if (!validateCSVMetadata(metadata))
    {
        fprintf(stderr,
            "Invalid CSV metadata.\n");

        return false;
    }

    // 運動データ全体がCSV仕様に適合するか検査する。
    if (!validateMotionData(data))
    {
        return false;
    }

    // CSV出力先ファイルを開く。
    std::ofstream file(filename);


    // ファイルを開けなかった場合は処理を中止する。
    if (!file.is_open())
    {
        fprintf(stderr,
            "Cannot open output file: %s\n",
            filename);

        return false;
    }

    // 地域設定に依存しない数値出力形式を使用する。
    // 小数点をピリオドに固定し、桁区切りを使用しない。
    file.imbue(std::locale::classic());

    // 必須メタデータを出力する。
    file << "# format=" << metadata.format << "\n";
    file << "# version=" << metadata.version << "\n";

    // 省略可能なメタデータは、
    // 値が指定されている場合だけ出力する。
    if (!metadata.title.empty())
        file << "# title=" << metadata.title << "\n";

    if (!metadata.device.empty())
        file << "# device=" << metadata.device << "\n";

    if (!metadata.source.empty())
        file << "# source=" << metadata.source << "\n";

    if (!metadata.created_at.empty())
        file << "# created_at=" << metadata.created_at << "\n";

    // 公称サンプリング間隔 [ms]
    if (!metadata.nominal_interval_ms.empty())
        file << "# nominal_interval_ms="
        << metadata.nominal_interval_ms << "\n";

    // 音声周波数および振幅の補間方式
    file << "# interpolation_audio_freq="
        << metadata.interpolation_audio_freq << "\n";

    file << "# interpolation_audio_amplitude="
        << metadata.interpolation_audio_amplitude << "\n";

    // 必須列を出力する。
    // 時刻 [ms]、XYZ座標 [mm]
    file << "time_ms,x_mm,y_mm,z_mm";

    // 省略可能な列は、存在する場合だけ出力する。
    // ジンバル角度 [rad]
    if (columns.gimbal1_rad >= 0)
        file << ",gimbal1_rad";

    if (columns.gimbal2_rad >= 0)
        file << ",gimbal2_rad";

    if (columns.gimbal3_rad >= 0)
        file << ",gimbal3_rad";

    // ボタン状態（ビットマスク）
    if (columns.buttons >= 0)
        file << ",buttons";

    // 音声周波数 [Hz]
    if (columns.audio_freq_hz >= 0)
        file << ",audio_freq_hz";

    // 音声振幅（無次元、0.0～1.0）
    if (columns.audio_amplitude >= 0)
        file << ",audio_amplitude";

    // ヘッダ行を終了する。
    file << "\n";

    // double型の値を再読み込み時に復元できるよう、
    // 必要な有効桁数を指定する。
    file << std::setprecision(
        std::numeric_limits<double>::max_digits10);

    // 全サンプルを1行ずつ出力する。
    for (const MotionSample& sample : data)
    {
        // 必須項目：時刻 [ms]、XYZ座標 [mm]
        file << sample.time_ms
            << "," << sample.position[0]
            << "," << sample.position[1]
            << "," << sample.position[2];

        // ジンバル角度 [rad]
        if (columns.gimbal1_rad >= 0)
            file << "," << sample.gimbalAngles[0];

        if (columns.gimbal2_rad >= 0)
            file << "," << sample.gimbalAngles[1];

        if (columns.gimbal3_rad >= 0)
            file << "," << sample.gimbalAngles[2];

        // ボタン状態（ビットマスク）
        if (columns.buttons >= 0)
            file << "," << sample.buttons;

        // 音声周波数 [Hz]
        if (columns.audio_freq_hz >= 0)
            file << "," << sample.audio_freq_hz;

        // 音声振幅（無次元、0.0～1.0）
        if (columns.audio_amplitude >= 0)
            file << "," << sample.audio_amplitude;

        // サンプル行を終了する。
        file << "\n";
    }

    // ファイルへの書き込み結果を検査する。
    if (!file)
    {
        fprintf(stderr,
            "CSV write failed: %s\n",
            filename);

        return false;
    }

    // ファイルを閉じる。
    // バッファに残っているデータも書き出される。
    file.close();

    // ファイルクローズ時のエラーを検査する。
    if (file.fail())
    {
        fprintf(stderr,
            "CSV close failed: %s\n",
            filename);

        return false;
    }

    // すべての書き込み処理が正常に完了した。
    return true;
}

// 試験専用の一時ファイルを作成する。
// 既存のCSVファイルを上書きしないために使用する。
static bool createCSVTestFile(
    char(&filename)[MAX_PATH])
{
    // Windowsの一時フォルダのパスを取得する。
    char tempPath[MAX_PATH] = {};

    const DWORD length =
        GetTempPathA(MAX_PATH, tempPath);

    if (length == 0 || length >= MAX_PATH)
    {
        fprintf(stderr,
            "Cannot obtain temporary directory.\n");

        return false;
    }

    // 重複しない名前の空ファイルを作成する。
    if (GetTempFileNameA(
        tempPath, "RnR", 0, filename) == 0)
    {
        fprintf(stderr,
            "Cannot create temporary file.\n");

        return false;
    }

    return true;
}

// 2つの運動サンプルの全項目が一致するか検査する。
// CSV書き出し・再読み込みによる情報の欠落を検出する。
static bool equalMotionSample(
    const MotionSample& a,
    const MotionSample& b)
{
    // 時刻（ms）
    if (a.time_ms != b.time_ms)
        return false;

    for (int axis = 0; axis < 3; ++axis)
    {
        // 位置（mm）
        if (a.position[axis] != b.position[axis])
            return false;

        // ジンバル角（rad）
        if (a.gimbalAngles[axis] != b.gimbalAngles[axis])
            return false;
    }

    // ボタン状態
    if (a.buttons != b.buttons)
        return false;

    // 音声周波数（Hz）
    if (a.audio_freq_hz != b.audio_freq_hz)
        return false;

    // 音声振幅（0.0～1.0）
    if (a.audio_amplitude != b.audio_amplitude)
        return false;

    return true;
}

// 2つの運動データについて、点数と全サンプルを比較する。
// 不一致があれば、その位置を表示してfalseを返す。
static bool equalMotionData(
    const std::vector<MotionSample>& a,
    const std::vector<MotionSample>& b)
{
    // サンプル数の一致を確認する。
    if (a.size() != b.size())
    {
        fprintf(stderr,
            "Sample count mismatch: %zu != %zu\n",
            a.size(), b.size());

        return false;
    }

    // 全サンプルを順番に比較する。
    for (size_t i = 0; i < a.size(); ++i)
    {
        if (!equalMotionSample(a[i], b[i]))
        {
            fprintf(stderr,
                "Motion data mismatch at sample %zu.\n",
                i + 1);

            return false;
        }
    }

    return true;
}

// 2つのCSVメタデータの全項目が一致するか検査する。
static bool equalMotionCSVMetadata(
    const MotionCSVMetadata& a,
    const MotionCSVMetadata& b)
{
    return
        a.format == b.format &&
        a.version == b.version &&
        a.title == b.title &&
        a.device == b.device &&
        a.source == b.source &&
        a.created_at == b.created_at &&
        a.nominal_interval_ms == b.nominal_interval_ms &&
        a.interpolation_audio_freq ==
        b.interpolation_audio_freq &&
        a.interpolation_audio_amplitude ==
        b.interpolation_audio_amplitude;
}

// 省略可能な列の有無が一致するか検査する。
// 列番号や列の並び順は比較しない。
static bool equalMotionCSVColumns(
    const MotionCSVColumns& a,
    const MotionCSVColumns& b)
{
    return
        (a.gimbal1_rad >= 0) == (b.gimbal1_rad >= 0) &&
        (a.gimbal2_rad >= 0) == (b.gimbal2_rad >= 0) &&
        (a.gimbal3_rad >= 0) == (b.gimbal3_rad >= 0) &&
        (a.buttons >= 0) == (b.buttons >= 0) &&
        (a.audio_freq_hz >= 0) == (b.audio_freq_hz >= 0) &&
        (a.audio_amplitude >= 0) == (b.audio_amplitude >= 0);
}

// 試験用：ファイルの全内容をバイト列として取得する。
static bool readFileBytes(
    const char* filename,
    std::string& content)
{
    std::ifstream file(
        filename, std::ios::binary);

    if (!file)
    {
        return false;
    }

    content.clear();

    char c;

    while (file.get(c))
    {
        content.push_back(c);
    }

    return file.eof() && !file.bad();
}

// 異常データの拒否と既存ファイルの保全を検査する。
static bool testRejectedWrite(
    const char* filename,
    const std::vector<MotionSample>& data,
    const MotionCSVMetadata& metadata,
    const MotionCSVColumns& columns)
{
    // 書き出し前のファイル内容
    std::string before;

    if (!readFileBytes(filename, before))
    {
        fprintf(stderr, "Cannot read original file.\n");
        return false;
    }

    // 異常データの書き出しを試みる。
    const bool rejected = !writeMotionCSV(
        filename, data, metadata, columns);

    // 書き出し後のファイル内容
    std::string after;

    if (!readFileBytes(filename, after))
    {
        fprintf(stderr, "Cannot read file after test.\n");
        return false;
    }

    // ファイル内容が変化していないことを確認する。
    const bool unchanged = (before == after);

    if (!unchanged)
    {
        fprintf(stderr, "File content changed.\n");
    }

    return rejected && unchanged;
}

// Touchを使用せずにCSV書き出し処理を検証する。
// 入力ファイルは指定する。
// 出力ファイルは関数内部で自動生成する
static bool testWriteMotionCSV(
    const char* inputFilename)
    try
{
    // 入力CSVを読み込み、形式を検査する。
    if (!readMotionCSV(inputFilename))
    {
        fprintf(stderr,
            "CSV write test: input error.\n");

        return false;
    }

    // 再読み込みによってrawDataなどが更新されるため、
// 比較用に書き出し前のデータを保存する。
    const std::vector<MotionSample> originalData = rawData;
    const MotionCSVMetadata originalMetadata = rawMetadata;
    const MotionCSVColumns originalColumns = rawColumns;

    // 試験専用の出力ファイルを確保する。
    char outputFilename[MAX_PATH] = {};

    if (!createCSVTestFile(outputFilename))
    {
        return false;
    }

    // 読み込んだ運動データをCSV形式で書き出す。
    bool result = writeMotionCSV(
        outputFilename,
        rawData,
        rawMetadata,
        rawColumns);

    // 書き出したCSVを再読み込みする。
    if (result)
    {
        result = readMotionCSV(outputFilename);
    }

    // 運動データ全体を比較する。
    if (result)
    {
        result = equalMotionData(
            originalData, rawData);
    }

    // メタデータを比較する。
    if (result)
    {
        result = equalMotionCSVMetadata(
            originalMetadata, rawMetadata);

        if (!result)
        {
            fprintf(stderr,
                "CSV metadata mismatch.\n");
        }
    }

    // 省略可能な列の有無を比較する。
    if (result)
    {
        result = equalMotionCSVColumns(
            originalColumns, rawColumns);

        if (!result)
        {
            fprintf(stderr,
                "CSV columns mismatch.\n");
        }
    }

    // 書き出し結果を表示する。
    fprintf(stderr, "Result: %s\n",
        result ? "PASS" : "FAIL");

    if (result)
    {
        fprintf(stderr, "Output file: %s\n",
            outputFilename);

        fprintf(stderr, "Samples: %zu\n",
            rawData.size());
    }

    // 試験成功時は一時ファイルを削除する。
    // 詳細表示モードでは内容確認のために残す。
    if (result && !VerboseMode)
    {
        if (!DeleteFileA(outputFilename))
        {
            fprintf(stderr,
                "Cannot delete temporary file: %s\n",
                outputFilename);
        }
    }

    return result;
}
catch (const std::bad_alloc&)
{
    // メモリ確保に失敗した場合は試験を中止する。
    fprintf(stderr,
        "Memory allocation failed. CSV write test cancelled.\n");

    return false;
}

// CSV書き出し関数の単体試験。
// CSV読み込みおよびTouchの初期化は行わない。
static bool testWriteMotionCSVUnit()
{
    // 正常なメタデータを生成する。
    MotionCSVMetadata metadata;
    metadata.format = "RnR";
    metadata.version = "1.0";
    metadata.source = "generated";
    metadata.nominal_interval_ms = "10";

    // 必須列のみを使用する。
    MotionCSVColumns columns;
    columns.time_ms = 0;
    columns.x_mm = 1;
    columns.y_mm = 2;
    columns.z_mm = 3;

    // 正常な運動データを3点生成する。
    std::vector<MotionSample> data(3);

    for (size_t i = 0; i < data.size(); ++i)
    {
        // 時刻 [ms]
        data[i].time_ms = i * 10;

        // 位置 [mm]
        data[i].position[0] =
            static_cast<double>(i);
    }

    // 試験専用の一時ファイルを確保する。
    char filename[MAX_PATH] = {};

    if (!createCSVTestFile(filename))
    {
        return false;
    }

    // W01：正常データの書き出し
    bool result = writeMotionCSV(
        filename, data, metadata, columns);

    fprintf(stderr, "W01: %s\n",
        result ? "PASS" : "FAIL");

    // 異常系試験用のメタデータ
    MotionCSVMetadata invalidMetadata;

    // W02：不正なフォーマット名
    invalidMetadata = metadata;
    invalidMetadata.format = "Invalid";

    bool passed = testRejectedWrite(
        filename, data, invalidMetadata, columns);

    fprintf(stderr, "W02: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W03：未対応のバージョン
    invalidMetadata = metadata;
    invalidMetadata.version = "2.0";

    passed = testRejectedWrite(
        filename, data, invalidMetadata, columns);

    fprintf(stderr, "W03: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W04：不正な公称サンプリング間隔 [ms]
    invalidMetadata = metadata;
    invalidMetadata.nominal_interval_ms = "0";

    passed = testRejectedWrite(
        filename, data, invalidMetadata, columns);

    fprintf(stderr, "W04: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W05：未対応の補間方式
    invalidMetadata = metadata;
    invalidMetadata.interpolation_audio_freq = "cubic";

    passed = testRejectedWrite(
        filename, data, invalidMetadata, columns);

    fprintf(stderr, "W05: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // 異常系試験用の運動データ
    std::vector<MotionSample> invalidData;

    // W06：運動データが空
    invalidData.clear();

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W06: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W07：開始時刻が0 msではない
    invalidData = data;
    invalidData[0].time_ms = 1;

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W07: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W08：時刻の重複
    invalidData = data;
    invalidData[1].time_ms = 0;

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W08: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W09：時刻の逆転
    invalidData = data;
    invalidData[2].time_ms = 5;

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W09: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W10：位置 [mm] がNaN
    invalidData = data;
    invalidData[1].position[0] =
        std::numeric_limits<double>::quiet_NaN();

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W10: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W11：位置 [mm] が無限大
    invalidData = data;
    invalidData[1].position[0] =
        std::numeric_limits<double>::infinity();

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W11: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W12：ジンバル角度 [rad] が無限大
    invalidData = data;
    invalidData[1].gimbalAngles[0] =
        std::numeric_limits<double>::infinity();

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W12: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W13：不正なボタン状態
    invalidData = data;
    invalidData[1].buttons = 4;

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W13: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W14：音声周波数 [Hz] が負
    invalidData = data;
    invalidData[1].audio_freq_hz = -1.0;

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W14: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W15：音声振幅が負
    invalidData = data;
    invalidData[1].audio_amplitude = -0.1;

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W15: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W16：音声振幅が上限を超える
    invalidData = data;
    invalidData[1].audio_amplitude = 1.1;

    passed = testRejectedWrite(
        filename, invalidData, metadata, columns);

    fprintf(stderr, "W16: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;

    // W17：出力ファイルを開けない場合
// Windowsのファイル名に使用できない文字を指定する。
    passed = !writeMotionCSV(
        "invalid<filename>.csv",
        data, metadata, columns);

    fprintf(stderr, "W17: %s\n",
        passed ? "PASS" : "FAIL");

    result = result && passed;



    // 成功時は一時ファイルを削除する。
    if (result && !VerboseMode)
    {
        DeleteFileA(filename);
    }
    else
    {
        fprintf(stderr, "Output file: %s\n",
            filename);
    }

    return result;
}

void writeResampledReplayData() {
    int t;
    FILE* fp_replay_resample;
    // write to file
    fp_replay_resample = fopen(FNAME_REPLAY_RESAMPLE, "w");
    if (fp_replay_resample== NULL) {
        fprintf(stderr, "file open failed! : %s \n", FNAME_REPLAY_RESAMPLE);
        exit(1);
    }
    else {
        fprintf(fp_replay_resample, "# time[ms], position x, y, z [mm], gimbal_angle [rad] \n");
        for (t = 0; t < end_idx_resample; ++t) {
            int i;
            fprintf(fp_replay_resample, "%d, ", replay_timeidx_resample[t]);
            for (i = 0; i < 3; i++)
            {
                fprintf(fp_replay_resample, "%f", replay_position_resample[t][i]);
               if (i < 2) {
                   fprintf(fp_replay_resample, ", ");

            }

            }
            fprintf(fp_replay_resample, "\n");

        }
        fclose(fp_replay_resample);
    }
}




 

/*******************************************************************************
 Main function.
 Initializes the device, starts the schedule, creates a schedule callback
 to handle gravity well forces, waits for the user to press a button, exits
 the application.
*******************************************************************************/
int main(int argc, char* argv[])
{
    const char* replayFilename = NULL;
    bool autoPlay = false;

    // CSV読み込み単体試験モード
    bool testCSV = false;

    // Touchを使用しないリサンプリング検証モード。
    bool testResample = false;

    // Touchを使用しないCSV書き出し試験モード。
    bool testWriteCSV = false;

    // CSV書き出し関数の単体試験モード
    bool testWriteCSVUnit = false;

for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--file") == 0 && i + 1 < argc) {
        replayFilename = argv[++i];
    }
    else if (strcmp(argv[i], "--play") == 0) {
        autoPlay = true;
    }
    else if (strcmp(argv[i], "--lang") == 0 && i + 1 < argc) {
        Language = argv[++i];
    }

 // --verbose が指定された場合、詳細表示モードを有効にする。
// VerboseModeは初期値falseなので、指定がなければ通常モードになる。
    else if (strcmp(argv[i], "--verbose") == 0) {
        VerboseMode = true;
    }

    // CSV読み込み単体試験モードを有効にする。
    else if (strcmp(argv[i], "--test-csv") == 0) {
        testCSV = true;
    }

    else if (strcmp(argv[i], "--test-resample") == 0)
    {
        testResample = true;
    }

    else if (strcmp(argv[i], "--test-write-csv") == 0)
    {
        testWriteCSV = true;
    }
    else if (strcmp(argv[i], "--test-write-csv-unit") == 0)
    {
        testWriteCSVUnit = true;
    }

}

// 言語設定は開発時の確認情報なので、Verboseモードでのみ表示する。
// 通常モードではNVDAによる不要な読み上げを抑える。
if (VerboseMode) {
    printf("Language: %s\n", Language);
}

// CSV単体試験モードではTouchを初期化せず、
// CSV読み込み処理だけを実行して終了する。
if (testCSV)
{
    // ファイル指定がなければ既定の記録CSVを使用する。
    const char* testFilename =
        (replayFilename != NULL)
        ? replayFilename
        : FNAME_REC;

    testReadMotionCSV(testFilename);

    return RawDataReady ? 0 : 1;
}

// Touchの初期化を行わずに、
// CSV読み込みとリサンプリングを検証する。
if (testResample)
{
    const char* filename =
        replayFilename != NULL
        ? replayFilename
        : FNAME_REC;

    return testResampleCSV(filename) ? 0 : 1;
}

// Touchを初期化せずにCSV書き出しを検証する。
if (testWriteCSV)
{
    const char* filename =
        replayFilename != NULL
        ? replayFilename
        : FNAME_REC;

    return testWriteMotionCSV(filename) ? 0 : 1;
}

// Touchを初期化せずにCSV書き出し関数を単体試験する。
if (testWriteCSVUnit)
{
    return testWriteMotionCSVUnit() ? 0 : 1;
}

HDErrorInfo error;

/* Initialize the device, must be done before attempting to call any hd
       functions. Passing in HD_DEFAULT_DEVICE causes the default device to be 
       initialized. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

    // 接続デバイスの型番は開発・診断用の情報として扱う。
 // 通常モードでは起動時の読み上げを減らすため表示しない。
 // RnRの起動を示すタイトルは、どちらのモードでも表示する。
    if (strcmp(Language, "ja") == 0) {
        printf("RnR ガイド・記録モーションデモ\n");

        if (VerboseMode) {
            printf("接続されたデバイス: %s\n",
                hdGetString(HD_DEVICE_MODEL_TYPE));
        }
    }
    else {
        printf("Guide and Record motion Demo!\n");

        if (VerboseMode) {
            printf("Found device model: %s.\n",
                hdGetString(HD_DEVICE_MODEL_TYPE));
        }
    }


    if (!initDemo())
    {
        // キャリブレーションに失敗した場合は、
        // 安全のためRnRの動作を開始しない。

        if (strcmp(Language, "ja") == 0) {
            printf("デバイスの初期化に失敗しました。\n");
            printf("RnRを終了します。\n");
        }
        else {
            printf("Device initialization failed.\n");
            printf("RnR will exit.\n");
        }

        // デバイスを解放する
        hdDisableDevice(hHD);

        // プログラムを異常終了する
        return -1;
    }
    /* Schedule the main callback that will render forces to the device. */
    hGravityWell = hdScheduleAsynchronous(
        guideAndRecordCallback, 0, 
        HD_MAX_SCHEDULER_PRIORITY);

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    // Windowsの音声出力デバイスを初期化する
  // 音声デバイスの初期化失敗は操作に影響するため、
// Verboseの設定にかかわらず、選択言語で通知する。
    if (!InitAudio()) {
        if (strcmp(Language, "ja") == 0) {
            fprintf(stderr, "音声デバイスの初期化に失敗しました。\n");
        }
        else {
            fprintf(stderr, "Failed to initialize audio device.\n");
        }
    }
    else {
        // 音声初期化の成功情報は診断用としてVerbose時のみ表示する。
        // 初期化失敗や準備失敗は、通常モードでも必ず通知する。
        if (VerboseMode) {
            if (strcmp(Language, "ja") == 0) {
                fprintf(stderr, "音声デバイスの初期化に成功しました。\n");
            }
            else {
                fprintf(stderr, "Audio device initialized successfully.\n");
            }
        }

        // 440Hzの音声データを登録する。
        if (PrepareAudio()) {
            if (VerboseMode) {
                if (strcmp(Language, "ja") == 0) {
                    fprintf(stderr, "音声データの準備に成功しました。\n");
                }
                else {
                    fprintf(stderr, "Audio data prepared successfully.\n");
                }
            }
        }
        else {
            if (strcmp(Language, "ja") == 0) {
                fprintf(stderr, "音声データの準備に失敗しました。\n");
            }
            else {
                fprintf(stderr, "Failed to prepare audio data.\n");
            }
        }
    }


    // 起動時にはヘルプ全文を表示せず、呼び出し方法だけを案内する。
    // NVDAによる長い読み上げを避け、必要なときだけHキーで確認できるようにする。
    // Verboseモードでもヘルプ全文は自動表示しない。
    if (strcmp(Language, "ja") == 0) {
        printf("操作方法を確認するにはHキーを押してください。\n");
    }
    else {
        printf("Press H for help.\n");
    }

    if (replayFilename != NULL) {
        if (read_resampled_ReplayData(replayFilename) && autoPlay) {
            ReplayDeviceState();
        }
    }

    /* Start the main application loop */
    if (!autoPlay) {
        mainLoop();
    }

    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hGravityWell);

    /* Disable the device. */
    hdDisableDevice(hHD);

    // 音声デバイスを解放する
    CloseAudio();

    return 0;
}
/******************************************************************************
 The main loop of execution.  Detects and interprets keypresses.  Monitors and 
 initiates error recovery if necessary.
******************************************************************************/
void mainLoop()
{
    int keypress;

    HDint previousButtons = 0;
    DeviceStateStruct buttonState = {};

    while (TRUE)
    {
        // Record処理から戻ったら、ボタン状態を更新する。
        // 記録終了時の押下を、新しい開始操作と誤認しないため。
        hdScheduleSynchronous(
            GetDeviceStateCallback,
            &buttonState,
            HD_DEFAULT_SCHEDULER_PRIORITY
        );

        HDint currentButtons = buttonState.buttons;

        // ボタン2の押下瞬間を検出する
        if ((currentButtons & HD_DEVICE_BUTTON_2) &&
            !(previousButtons & HD_DEVICE_BUTTON_2))
        {
            //printf("Button 2 detected.\n");

            if (!StartAudio()) {
                fprintf(stderr, "音声再生の開始に失敗しました。\n");
            }
            else {
                //printf("StartAudio succeeded.\n");
            }
        }


        // ボタン2の解放瞬間を検出する
        if (!(currentButtons & HD_DEVICE_BUTTON_2) &&
            (previousButtons & HD_DEVICE_BUTTON_2))
        {
            StopAudio();
        }


        // ボタン1の押下瞬間を検出
        if ((currentButtons & HD_DEVICE_BUTTON_1) &&
            !(previousButtons & HD_DEVICE_BUTTON_1))
        {
            printf("Button 1: Record start\n");

            // 記録開始から終了、CSV保存まで実行する。
            // 終了後はヘルプを自動表示せず、記録結果の通知だけを残す。
            // ヘルプは必要なときにHキーで呼び出せる。
            RecordDeviceState();

            // 記録終了後のボタン状態を再取得
            hdScheduleSynchronous(
                GetDeviceStateCallback,
                &buttonState,
                HD_DEFAULT_SCHEDULER_PRIORITY
            );

            // 次の押下判定に備える
            previousButtons = buttonState.buttons;
        }
        else {
            // 通常時は今回の状態を次回比較用に保存
            previousButtons = currentButtons;
        }

        // ボタン2が押されていて、音声再生が完了していたら
// 同じ440Hzの音声データを再生する
        if ((currentButtons & HD_DEVICE_BUTTON_2) &&
            IsAudioFinished())
        {
            if (!StartAudio()) {
                fprintf(stderr, "音声の連続再生に失敗しました。\n");
            }
        }

        if (_kbhit())
        {
            keypress = getch();
            keypress = toupper(keypress);
            
            // 各コマンド実行後のヘルプ自動表示を廃止する。
            // NVDAによる繰り返し読み上げを防ぐため、
            // ヘルプ全文はHキーが押された場合にのみ表示する。
            // Record・Replayなどの処理内容は変更しない。
            switch (keypress)
            {
            case 'L': read_resampled_ReplayData(FNAME_REPLAY_RESAMPLE); break;
            case 'G': ReplayDeviceState(); break;
            case 'R': RecordDeviceState(); break;
            case 'D': read_RecordData(); break;

            case 'P': PrintDeviceState(FALSE); break;
            case 'C': PrintDeviceState(TRUE); break;
                // 新しいリサンプリング処理が完成するまでは、
                // RAWデータの読み込みだけを行う。
            case 'A': read_raw_ReplayData(); break;
                // 新しいCSV書き出し処理が完成するまでは、
                // 旧固定長配列からの書き出しを禁止する。
            case 'W':
                if (strcmp(Language, "ja") == 0)
                {
                    fprintf(stderr,
                        "現在、CSV書き出し機能は停止中です。\n");
                }
                else
                {
                    fprintf(stderr,
                        "CSV export is temporarily disabled.\n");
                }
                break;


                // ヘルプは利用者が明示的に要求した場合だけ表示する。
            case 'H': PrintHelp(); break;
            case 'Q': return;

                // 未定義キーではヘルプを自動表示しない。
            default: break;
            }
        }

        /* Check if the scheduled callback has stopped running */
        if (!hdWaitForCompletion(hGravityWell, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            return;
        }
    }
}

/*******************************************************************************
 Servo callback.  
 Called every servo loop tick.  Simulates a gravity well, which sucks the device 
 towards its center whenever the device is within a certain range.
*******************************************************************************/
HDCallbackCode HDCALLBACK guideAndRecordCallback(void *data)
{
    const HDdouble kStiffness = 0.15; /* N/mm */ //magnitude of gravity. default is 0.075
    const HDdouble kStylusTorqueConstant = 500; /* torque spring constant (mN.m/radian)*/
    const HDdouble kJointTorqueConstant = 1000; /* torque spring constant (mN.m/radian)*/
 
    const HDdouble kForceInfluence = 100; /* mm */
    const HDdouble kTorqueInfluence = 3.14; /* radians */

    /* This is the position of the gravity well in cartesian
       (i.e. x,y,z) space. */
    //static const hduVector3Dd wellPos(0,0,0);
    static const hduVector3Dd stylusVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    static const hduVector3Dd jointVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    
    HDErrorInfo error;
    hduVector3Dd position;

    hduVector3Dd force;
    hduVector3Dd positionTwell;
    hduVector3Dd gimbalAngles;
    hduVector3Dd gimbalTorque;
    hduVector3Dd gimbalAngleOfTwist;
    hduVector3Dd jointAngles;
    hduVector3Dd jointTorque;
    hduVector3Dd jointAngleOfTwist;

    HHD hHD = hdGetCurrentDevice();

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    hdBeginFrame(hHD);

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES,gimbalAngles );
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,jointAngles );

    memset(force, 0, sizeof(hduVector3Dd));
    

    /* >  positionTwell = wellPos-position  < 
       Create a vector from the device position towards the gravity 
       well's center. */
    hduVecSubtract(positionTwell, wellPos, position);
    
    
    /* If the device position is within some distance of the gravity well's 
       center, apply a spring force towards gravity well's center.  The force
       calculation differs from a traditional gravitational body in that the
       closer the device is to the center, the less force the well exerts;
       the device behaves as if a spring were connected between itself and
       the well's center. */
    if (hduVecMagnitude(positionTwell) < kForceInfluence)
    {
        /* >  F = k * x  < 
           F: Force in Newtons (N)
           k: Stiffness of the well (N/mm)
           x: Vector from the device endpoint position to the center 
           of the well. */
        hduVecScale(force, positionTwell, kStiffness);
    }
       

    /* Send the forces & torques to the device. */
    /*Switch back between sending forces & torques 
    to the base motors */
    if (!ReplayMode) {
        force[0] = force[1] = force[2] = 0;
    }
    hdSetDoublev(HD_CURRENT_FORCE, force);
    
    
//    printf("%f\t%f\t%f\n\n", jointTorque[0], jointTorque[1], jointTorque[2]);
    /* End haptics frame. */
    hdEndFrame(hHD);

    /* Check for errors and abort the callback if a scheduler error
       is detected. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, 
                      "Error detected while rendering gravity well\n");
        
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}

bool initDemo(void)
{
    HDErrorInfo error;
    int calibrationStyle;

    // 現在のキャリブレーション状態を取得する
    HDenum calibrationStatus = hdCheckCalibration();

    // Touchが対応しているキャリブレーション方式を取得する
    hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);

    // 対応するキャリブレーション方式は診断用情報として扱う。
    // 通常モードでは表示せず、NVDAによる不要な読み上げを抑える。
    // 方式の取得とキャリブレーション処理自体は変更しない。
    if (VerboseMode) {
        printf("Calibration styles:\n");

        if (calibrationStyle & HD_CALIBRATION_AUTO) {
            printf("  AUTO\n");
        }

        if (calibrationStyle & HD_CALIBRATION_INKWELL) {
            printf("  INKWELL\n");
        }

        if (calibrationStyle & HD_CALIBRATION_ENCODER_RESET) {
            printf("  ENCODER_RESET\n");
        }
    }


    // キャリブレーション状態の詳細は診断用情報として扱う。
   // 通常モードでは状態コードを表示せず、必要な操作案内だけを残す。
   // 未知の状態も診断時に確認できるよう、Verboseでは表示する。
    if (VerboseMode) {
        switch (calibrationStatus)
        {
        case HD_CALIBRATION_OK:
            printf("Calibration status: OK\n");
            break;

        case HD_CALIBRATION_NEEDS_UPDATE:
            printf("Calibration status: NEEDS_UPDATE\n");
            break;

        case HD_CALIBRATION_NEEDS_MANUAL_INPUT:
            printf("Calibration status: NEEDS_MANUAL_INPUT\n");
            break;

        default:
            printf("Calibration status: UNKNOWN (%d)\n",
                calibrationStatus);
            break;
        }
    }

    // 現在のキャリブレーション状態を確認
    //HDenum calibrationStatus = hdCheckCalibration();

    // キャリブレーション済みなら、そのまま終了する
    if (calibrationStatus == HD_CALIBRATION_OK) {
        if (strcmp(Language, "ja") == 0) {
            printf("キャリブレーション済みです。\n");
        }
        else {
            printf("Device is already calibrated.\n");
        }

        return true;
    }

    // 以下は既存の処理をそのまま残す
    if (strcmp(Language, "ja") == 0) {
        printf("キャリブレーションを行います。\n");
    }
    else {
        printf("Calibration\n");
    }

    hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);

    if (calibrationStyle & HD_CALIBRATION_INKWELL)
    {
        // INKWELL方式では、スタイラスをインクウェルに戻して
        // キャリブレーションが完了するのを待つ。

        if (strcmp(Language, "ja") == 0) {
            printf("キャリブレーションが必要です。\n");
            printf("スタイラスをインクウェルにカチッとはめ込んでください。\n");
            printf("完了するまでお待ちください。\n");
        }
        else {
            printf("Calibration is required.\n");
            printf("Place the stylus firmly into the inkwell until it clicks into place.\n");
            printf("Please wait for calibration to complete.\n");
        }

        // 最大30秒間、キャリブレーション状態を確認する。
        for (int i = 0; i < 300; i++)
        {
            if (hdCheckCalibration() == HD_CALIBRATION_OK)
            {
                if (strcmp(Language, "ja") == 0) {
                    printf("キャリブレーションが完了しました。\n");
                }
                else {
                    printf("Calibration complete.\n");
                }

                return true;
            }

            // 100ミリ秒待ってから再確認する。
            Sleep(100);
        }

        // 30秒経過しても完了しなかった場合
        if (strcmp(Language, "ja") == 0) {
            printf("キャリブレーションが完了しませんでした。\n");
        }
        else {
            printf("Calibration did not complete.\n");
        }

        return false;
    }

    if (calibrationStyle & HD_CALIBRATION_ENCODER_RESET )
    {
        if (strcmp(Language, "ja") == 0) {
            printf("デバイスをリセット位置に置いてください。\n");
            printf("準備ができたら、何かキーを押してください。\n");
        }
        else {
            printf("Please prepare for starting the demo by \n");
            printf("placing the device at its reset position.\n\n");
            printf("Press any key to continue...\n");
        }

        getch();

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            if (strcmp(Language, "ja") == 0) {
                printf("キャリブレーションが完了しました。\n\n");
            }
            else {
                printf("Calibration complete.\n\n");
            }
            return 1;
        }
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            return 0;           
        }
    }
}
/*****************************************************************************/
