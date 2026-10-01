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

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
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

static bool ReplayMode = false;
static bool ReplayReady = false; // whether replay data loaded or not.
/* # of record */
#define LEN_REC 10000
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
// re-sampling interval[ms] for replay data
#define INTV_RESAMPLE_REPLAY_MS 10
// Maximum length of replay data (resample)
#define MAX_LEN_REPLAY_RESAMPLE 10000

#define BUFSIZE MAX_PATH


static int end_idx_resample;
static int end_idx_record;


static hduVector3Dd rec_position[LEN_REC];
static hduVector3Dd rec_gimbalAngles[LEN_REC];
static DWORD rec_timeidx[LEN_REC];
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
    static const char help[] = {\
"GuideAndRecord Help\n\
---\n\
L: Load replay data\n\
G: Guide mode start\n\
R: Record mode start\n\
D: Load Recorded data\n\
P: Prints device state\n\
C: Continuously prints device state\n\
A: Load and resample RAW replay data \n\
W: Write resampled replay data(debug)\n\
H: Prints help menu\n\
Q: Quits the program\n\
---"};
    
    printf("\n%s\n", help);
    PrintCwd();
}

/* Synchronization structure. */
 typedef struct
{
    HDdouble position[3]; /* mm */
    HDdouble gimbalAngles[3];  /* rad */       
} DeviceStateStruct;

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData)
{
    DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;

    hdGetDoublev(HD_CURRENT_POSITION, pState->position);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, pState->gimbalAngles);

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
    /* # of record */
    // #define LEN_REC 10000
    /* interval for recording [ms]*/
    // #define INTV_REC 10
    float lim_rec_time = LEN_REC * INTV_REC * 0.001;
    int i;
    int t;
    double end_time;
    DWORD now, time_next;

    FILE* fp_rec_csv = NULL;

    DeviceStateStruct state;

    memset(&state, 0, sizeof(DeviceStateStruct));
    //end_idx_record = end_idx_resample;
    //end_time = end_idx_record * INTV_REC / 1000.0; //sec

    fprintf(stderr, "Now I record your motion in %f seconds.\n", lim_rec_time);


    fprintf(stderr, "Recording mode starts after 3 sec.count.");
    for (i = 3; i > 0; --i) {
        fprintf(stderr, "%d   ", i);
        Sleep(1000);
    }
    fprintf(stderr,"...go!\n");
    fprintf(stderr,"### hit any key to end record ###\n");

    now = time_start = timeGetTime();

    hdScheduleSynchronous(GetDeviceStateCallback, &state,
        HD_DEFAULT_SCHEDULER_PRIORITY);

    memcpy(rec_position[0], state.position, sizeof(hduVector3Dd));
    memcpy(rec_gimbalAngles[0], state.gimbalAngles, sizeof(hduVector3Dd));
    rec_timeidx[0] = now;
    time_next = now;

    //for(t=1;t< lim_rec_time;++t)
    t = 1;
    do
    {
        time_next += INTV_REC;
        while (now < time_next) { //wait until next time
            now = timeGetTime();
        }
        hdScheduleSynchronous(GetDeviceStateCallback, &state,
            HD_DEFAULT_SCHEDULER_PRIORITY);

        memcpy(rec_position[t], state.position, sizeof(hduVector3Dd));
        memcpy(rec_gimbalAngles[t], state.gimbalAngles, sizeof(hduVector3Dd));
        rec_timeidx[t] = now;
        ++t;
    } while (!_kbhit() && t < LEN_REC); // exit when key press or time limit.
    
    end_idx_record = t;

//#if defined(WIN32)
//            Sleep(INTV_REC);
//#elif defined(linux)
//            struct timespec timeOut;
//            timeOut.tv_sec = 0;
//            timeOut.tv_nsec = 1 * 100000000;
//            nanosleep(&timeOut, NULL);
//#endif
 

    fprintf(stderr,"record end.\n");
    /* get time difference */
    for (t = 0; t < end_idx_record; ++t) {
        rec_timeidx[t] -= time_start;
    }
    
    for (t = 0; t < end_idx_record; ++t) {
        int i;
        printf("time:%d: ", rec_timeidx[t]);
        printf("Current position (mm):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", rec_position[t][i]);
        }
        //printf("\n");

        printf("Current Gimbal Angles (rad):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", rec_gimbalAngles[t][i]);
        }
        printf("\n");

    }
    // write to file
    fp_rec_csv = fopen(FNAME_REC, "w");
    if (fp_rec_csv==NULL) {
        fprintf(stderr, "file open failed! : %s \n",FNAME_REC);
        exit(1);
    }
    else {
        fprintf(fp_rec_csv, "# time[ms], position x, y, z [mm], gimbal_angle [rad] \n");
        for (t = 0; t < end_idx_record; ++t) {
            int i;
            fprintf(fp_rec_csv, "%d, ", rec_timeidx[t]);
            for (i = 0; i < 3; i++)
            {
                fprintf(fp_rec_csv, "%f, ", rec_position[t][i]);
            }
            for (i = 0; i < 3; i++)
            {
                fprintf(fp_rec_csv, "%f", rec_gimbalAngles[t][i]);
                if (i < 2) {
                    fprintf(fp_rec_csv, ", ");
                }
                
            }
            fprintf(fp_rec_csv, "\n");

        }
        fclose(fp_rec_csv);
    }
    fprintf(stderr, "record end.total time %d time steps.\n",end_idx_record);

}

/*****************************************************************************
 Callback that replay state.
*****************************************************************************/
void ReplayDeviceState()
{
    DWORD now,time_replay_start,time_next;

    int i;
    int i_resample;

    if (!ReplayReady) { //replay data not loaded successfully
        fprintf(stderr, "replay data not loaded successfully.\n");
        fprintf(stderr, "Please confirm replay data loaded by L command.\n");
        return;
    }

    // guide to initial gravity well position
    fprintf(stderr, "Replay mode :\n");
    fprintf(stderr, "   Please remove stylus from calibration well, and hold any position.\n");
    fprintf(stderr, "   When you got ready, press any key.");
    fflush(stdin);  getch();
    fprintf(stderr, "Now I guide you to the starting position.\n");
    
    wellPos[0] = replay_position_resample[0][0];
    wellPos[1] = replay_position_resample[0][1];
    wellPos[2] = replay_position_resample[0][2];
    ReplayMode = true;

    fprintf(stderr, "When your stylus moved to start position, press any key.\n");
    fflush(stdin);  getch();

    printf("Replay starts after 3 sec.count.");
    for (i = 3; i > 0; --i) {
        printf("%d   ", i);
        Sleep(1000);
    }
    printf("...go!\n");
  
    // replay 
    now = time_replay_start = timeGetTime();
    
    for (i_resample = 1; i_resample < (end_idx_resample-1); ++i_resample) {
        time_next = time_replay_start + replay_timeidx_resample[i_resample];

            while (now < time_next) { //wait until next time
                now = timeGetTime();
            }
            // move gravity well
            memcpy(wellPos, replay_position_resample[i_resample], sizeof(hduVector3Dd));


    }



    
    fprintf(stderr, "press any key to quit replay mode.\n");
    fflush(stdin); getch();
    ReplayMode = false;
    fprintf(stderr, "replay mode disabled.\n");
    return;
    }


void read_resampled_ReplayData() {
    FILE* fp_replay_csv = fopen(FNAME_REPLAY_RESAMPLE, "rt");
    if (!fp_replay_csv) {
        fprintf(stderr, "Cannot open file:%s\n", FNAME_REPLAY_RESAMPLE);
        exit(1);
    }
    char linebuf[LEN_LINE_CSV];
    int i = 0;
    int tmpTimeMsec;
    double tmpT, tmpX, tmpY, tmpZ;
    // read 1-line until EOF
    while (fgets(linebuf, LEN_LINE_CSV, fp_replay_csv) != NULL) { // until encount EOF

        if (linebuf[0] != '#') { // skip header line
            sscanf(linebuf, "%lf,%lf,%lf,%lf", &tmpT, &tmpX, &tmpY, &tmpZ);
            // convert time to msec
            tmpTimeMsec = tmpT;
            replay_timeidx_resample[i] = tmpTimeMsec;
            replay_position_resample[i][0] = tmpX;
            replay_position_resample[i][1] = tmpY;
            replay_position_resample[i][2] = tmpZ;

            ++i;
            if (i < MAX_LEN_REPLAY_RAW) {
                continue;
            }
            else {
                fprintf(stderr, "Maximum data length reached. reading file terminated. \n");
                break;
            }
        }
    }
    // store end of index for resample
    end_idx_resample = i + 1;
    fprintf(stderr, "Replay resampled data loaded. total %d elements.\n", end_idx_resample);
    ReplayReady = true;
    return;
}

void read_RecordData() {
    FILE* fp_replay_csv = fopen(FNAME_REC, "rt");
    if (!fp_replay_csv) {
        fprintf(stderr, "Cannot open file:%s\n", FNAME_REC);
        exit(1);
    }
    char linebuf[LEN_LINE_CSV];
    int i = 0;
    int tmpTimeMsec;
    double tmpT, tmpX, tmpY, tmpZ;
    // read 1-line until EOF
    while (fgets(linebuf, LEN_LINE_CSV, fp_replay_csv) != NULL) { // until encount EOF

        if (linebuf[0] != '#') { // skip header line
            sscanf(linebuf, "%lf,%lf,%lf,%lf", &tmpT, &tmpX, &tmpY, &tmpZ);
            // convert time to msec
            tmpTimeMsec = tmpT;
            replay_timeidx_resample[i] = tmpTimeMsec;
            replay_position_resample[i][0] = tmpX;
            replay_position_resample[i][1] = tmpY;
            replay_position_resample[i][2] = tmpZ;

            ++i;
            if (i < MAX_LEN_REPLAY_RAW) {
                continue;
            }
            else {
                fprintf(stderr, "Maximum data length reached. reading file terminated. \n");
                break;
            }
        }
    }
    // store end of index for resample
    end_idx_resample = i + 1;
    fprintf(stderr, "Replay resampled data loaded. total %d elements.\n", end_idx_resample);

    ReplayReady = true;
    return;
}


void read_raw_ReplayData() {
    FILE* fp_replay_csv = fopen(FNAME_REPLAY, "rt");
    if (!fp_replay_csv) {
        fprintf(stderr, "Cannot open file:%s\n", FNAME_REPLAY);
        exit(1);
    }
    char linebuf[LEN_LINE_CSV];
    int i = 0;
    int tmpTimeMsec;
    double tmpT,tmpX, tmpY, tmpZ;
    // read 1-line until EOF
    while (fgets(linebuf, LEN_LINE_CSV, fp_replay_csv) != NULL) { // until encount EOF

        if (linebuf[0] != '#') { // skip header line
            sscanf(linebuf, "%lf,%lf,%lf,%lf", &tmpT, &tmpX, &tmpY, &tmpZ);
            // convert time to msec
            tmpTimeMsec = tmpT * 1000;
            replay_timeidx_raw[i] = tmpTimeMsec;
            replay_position_raw[i][0] = tmpX;
            replay_position_raw[i][1] = tmpY;
            replay_position_raw[i][2] = tmpZ;

            ++i;
            if (i < MAX_LEN_REPLAY_RAW) {
                continue;
            }
            else {
                fprintf(stderr, "Maximum data length reached. reading file terminated. \n");
                break;
            }
        }
    }
    fprintf(stderr, "Replay raw data loaded. total %d elements.\n", i);
    ReplayReady = true;
    return;
}



void resampleReplayData() {
    int i = 0;
    int i_resample = 0;
    int current_time; //ms
    int offset_time; //ms

    if (!ReplayReady) { //replay data not loaded successfully
        fprintf(stderr, "replay data not loaded successfully.abort resampling.\n");
        return;
    }

    // set first data as timeidx 0
    offset_time = replay_timeidx_raw[0];
    current_time = offset_time;

    replay_timeidx_resample[0] = 0;
    memcpy(replay_position_resample[0], replay_position_raw[0], sizeof(hduVector3Dd));

    
    for (i_resample = 1; i_resample < MAX_LEN_REPLAY_RESAMPLE;++i_resample) {
        // set kijun time
        current_time += INTV_RESAMPLE_REPLAY_MS;
        //search next time idx which exceed kijun time
        while (replay_timeidx_raw[i] < current_time) {
            ++i;
            if (i >= MAX_LEN_REPLAY_RAW) {
                fprintf(stderr, "maximum raw replay data reached.\n");
                return;
            }
        }
        // set resampled array
        replay_timeidx_resample[i_resample] = replay_timeidx_raw[i];
        memcpy(replay_position_resample[i_resample], replay_position_raw[i], sizeof(hduVector3Dd));
        fprintf(stderr, "resample :index:%d  resampled time %d , raw time  %d.\n",
            i_resample, replay_timeidx_resample[i_resample], replay_timeidx_raw[i]);
        
        // store end of index for resample
        end_idx_resample = i_resample + 1;


    }
    return;
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
 //           for (i = 0; i < 3; i++)
 //           {
 //               fprintf(fp_rec_csv, "%f", rec_gimbalAngles[t][i]);
 //               if (i < 2) {
 //                   fprintf(fp_rec_csv, ", ");
 //               }

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

    printf("Guide and Record motion Demo!\n");
    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    if (!initDemo())
    {
        printf("Demo Initialization failed\n");
        printf("Press any key to exit\n");
        getch();
        
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

    PrintHelp();

    /* Start the main application loop */
    mainLoop();

    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hGravityWell);

    /* Disable the device. */
    hdDisableDevice(hHD);

    return 0;
}
/******************************************************************************
 The main loop of execution.  Detects and interprets keypresses.  Monitors and 
 initiates error recovery if necessary.
******************************************************************************/
void mainLoop()
{
    int keypress;

    while (TRUE)
    {
        if (_kbhit())
        {
            keypress = getch();
            keypress = toupper(keypress);
            
            switch (keypress)
            {
                case 'L': read_resampled_ReplayData(); PrintHelp(); break;
                case 'G': ReplayDeviceState(); PrintHelp(); break;
                case 'R': RecordDeviceState(); PrintHelp(); break;
                case 'D': read_RecordData(); PrintHelp(); break;

                case 'P': PrintDeviceState(FALSE); PrintHelp(); break;
                case 'C': PrintDeviceState(TRUE); PrintHelp(); break;
                case 'A': read_raw_ReplayData(); resampleReplayData(); PrintHelp(); break;
                case 'W': writeResampledReplayData(); PrintHelp(); break;
                case 'H': PrintHelp(); break;
                case 'Q': return;
                default: PrintHelp(); break;
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
    printf("Calibration\n");

    hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
    if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL)
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");
        getch();
        return 1;
    }
    if (calibrationStyle & HD_CALIBRATION_ENCODER_RESET )
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");

        getch();

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            printf("Calibration complete.\n\n");
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
