

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
//https://ftctechnh.github.io/ftc_app/doc/javadoc/allclasses-noframe.html

import java.lang.Object;

//import com.qualcomm.robotcore.hardware.ScannedDevices.MapAdapter;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.AnalogInput; //(implements import com.qualcomm.robotcore.hardware.HardwareDevice);
//import com.qualcomm.robotcore.hardware.AnalogOutput; //(implements import com.qualcomm.robotcore.hardware.HardwareDevice);
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer; //(implements android.hardware.SensorEventListener);
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope; //(implements android.hardware.SensorEventListener);
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation; //(implements android.hardware.SensorEventListener);
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
//import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeRegistrar;
import android.widget.BaseAdapter; //(implements android.widget.ListAdapter, android.widget.SpinnerAdapter);
import android.widget.ArrayAdapter;//<T> (implements android.widget.Filterable, android.widget.ThemedSpinnerAdapter);
//import com.qualcomm.ftccommon.FtcWifiDirectChannelSelectorActivity.WifiChannelItemAdapter;
//import com.qualcomm.ftccommon.FtcWifiDirectRememberedGroupsActivity.WifiP2pGroupItemAdapter;
import android.os.Binder; //(implements android.os.IBinder);
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.robotcore.hardware.Blinker.Step;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.ClassFactory.InstanceHolder;
import com.qualcomm.ftccommon.ClassManagerFactory;
import com.qualcomm.ftccommon.CommandList.CmdPlaySound;
import com.qualcomm.ftccommon.CommandList.CmdRequestSound;
import com.qualcomm.ftccommon.CommandList.CmdStopPlayingSounds;
import com.qualcomm.ftccommon.CommandList.CmdVisuallyIdentify;
import com.qualcomm.ftccommon.CommandList.LynxAddressChangeRequest;
import com.qualcomm.ftccommon.CommandList.LynxAddressChangeRequest.AddressChange;
//import com.qualcomm.ftccommon.CommandList.LynxFirmwareImagesResp;
//import com.qualcomm.ftccommon.CommandList.LynxFirmwareUpdate;
//import com.qualcomm.ftccommon.CommandList.LynxFirmwareUpdateResp;
//import com.qualcomm.ftccommon.CommandList.USBAccessibleLynxModulesRequest;
//import com.qualcomm.ftccommon.CommandList.USBAccessibleLynxModulesResp;
import android.content.Context;
import android.content.ContextWrapper;
import android.app.Service; //(implements android.content.ComponentCallbacks2);
import com.qualcomm.ftccommon.FtcRobotControllerService; //(implements import com.qualcomm.robotcore.eventloop.opmode.EventLoopManagerClient);
import com.qualcomm.robotcore.hardware.CRServoImpl; //(implements import com.qualcomm.robotcore.hardware.CRServo);
import com.qualcomm.robotcore.hardware.CRServoImplEx; //(implements import com.qualcomm.robotcore.hardware.PwmControl);
import com.qualcomm.robotcore.hardware.DcMotorImpl; //(implements import com.qualcomm.robotcore.hardware.DcMotor);
import com.qualcomm.robotcore.hardware.DcMotorImplEx; //(implements import com.qualcomm.robotcore.hardware.DcMotorEx);
//import RecvLoopRunnable.DegenerateCallback;
//import com.qualcomm.ftccommon.FtcLynxFirmwareUpdateActivity.ReceiveLoopCallback;
//import RecvLoopRunnable.DegenerateCallback;
//import com.qualcomm.ftccommon.FtcLynxModuleAddressUpdateActivity.ReceiveLoopCallback;
//import RecvLoopRunnable.DegenerateCallback;
//import com.qualcomm.ftccommon.FtcWifiDirectRememberedGroupsActivity.RecvLoopCallback;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl; //(implements import com.qualcomm.robotcore.hardware.DigitalChannel);

//EditActivity

import com.qualcomm.ftccommon.FtcLynxModuleAddressUpdateActivity;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.EventLoopManager; //(implements import com.qualcomm.robotcore.eventloop.SyncdDevice.Manager);
//import android.app.Fragment; //(implements android.content.ComponentCallbacks2, android.view.View.OnCreateContextMenuListener);
//import android.preference.PreferenceFragment;
import com.qualcomm.ftccommon.FtcAboutActivity.AboutFragment;
import com.qualcomm.ftccommon.FtcAdvancedRCSettingsActivity.SettingsFragment;
//import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity.SettingsFragment;

//Frame

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import com.qualcomm.ftccommon.FtcEventLoop.DefaultUsbModuleAttachmentHandler; //(implements import com.qualcomm.ftccommon.UsbModuleAttachmentHandler);
import com.qualcomm.ftccommon.FtcEventLoopBase; //(implements import com.qualcomm.robotcore.eventloop.EventLoop);
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopIdle;
//import com.qualcomm.ftccommon.FtcEventLoopBase.LynxUsbDeviceContainer;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
//import com.qualcomm.ftccommon.FtcLynxModuleAddressUpdateActivity.AddressAndDisplayName; //(implements java.lang.Comparable<T>);
//import com.qualcomm.ftccommon.FtcLynxModuleAddressUpdateActivity.AddressConfiguration;
//import com.qualcomm.ftccommon.FtcLynxModuleAddressUpdateActivity.DisplayedModule;
//import com.qualcomm.ftccommon.FtcLynxModuleAddressUpdateActivity.DisplayedModuleList;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealthImpl; //(implements import com.qualcomm.robotcore.hardware.HardwareDeviceHealth);
import com.qualcomm.robotcore.hardware.HardwareMap; //(implements java.lang.Iterable<T>);
import com.qualcomm.robotcore.hardware.HardwareMap.DeviceMapping; //<DEVICE_TYPE> (implements java.lang.Iterable<T>);
import com.qualcomm.robotcore.hardware.I2cAddr;
//import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;  //(implements import com.qualcomm.robotcore.hardware.I2cControllerPortDevice);
//import com.qualcomm.robotcore.hardware.I2cDeviceImpl;  // (implements import com.qualcomm.robotcore.hardware.HardwareDevice, import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyCallback, import com.qualcomm.robotcore.hardware.I2cDevice);
//import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.HeartbeatAction;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;  //<DEVICE_CLIENT> (implements import com.qualcomm.robotcore.hardware.HardwareDevice);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;  //<DEVICE_CLIENT,PARAMETERS>;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl.Callback;  // (implements import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyBeginEndNotifications, import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyCallback);
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl.WriteCacheStatus;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchReadHistoryImpl;  // (implements import com.qualcomm.robotcore.hardware.I2cDeviceSynchReadHistory);
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;  // (implements import com.qualcomm.robotcore.hardware.Engagable, import com.qualcomm.robotcore.hardware.I2cDeviceSynch);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;  // (implements import com.qualcomm.robotcore.hardware.I2cDeviceSynch);
import com.qualcomm.robotcore.hardware.IrSeekerSensor.IrSeekerIndividualSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.robotcore.hardware.LED;  // (implements import com.qualcomm.robotcore.hardware.HardwareDevice, import com.qualcomm.robotcore.hardware.SwitchableLight);
//import com.qualcomm.robotcore.hardware.LegacyModulePortDeviceImpl;  // (implements import com.qualcomm.robotcore.hardware.LegacyModulePortDevice);
import com.qualcomm.robotcore.hardware.LightBlinker;  // (implements import com.qualcomm.robotcore.hardware.Blinker);
import com.qualcomm.robotcore.hardware.LightMultiplexor;  // (implements import com.qualcomm.robotcore.hardware.SwitchableLight);
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode.LinearOpModeHelper;  // (implements java.lang.Runnable);
import com.qualcomm.robotcore.hardware.LynxModuleMeta;
import com.qualcomm.robotcore.hardware.LynxModuleMetaList;  // (implements java.lang.Iterable<T>);
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.ColumnMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.DenseMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.ColumnMajorMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.RowMajorMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.RowMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.SliceMatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;  // (implements android.hardware.SensorEventListener);
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;  // (implements import com.qualcomm.robotcore.hardware.PWMOutput);
import com.qualcomm.robotcore.hardware.PWMOutputImplEx;  // (implements import com.qualcomm.robotcore.hardware.PWMOutputEx);
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import com.qualcomm.robotcore.util.Range;

//RefCounted

//import com.qualcomm.ftccommon.SoundPlayer.SoundInfo;

//RobocolParsableBase

import com.qualcomm.robotcore.hardware.Gamepad;

//RobotCoreCommandList

import com.qualcomm.ftccommon.CommandList;
import com.qualcomm.robotcore.hardware.ScannedDevices;
import com.qualcomm.robotcore.util.SerialNumber;  // (implements java.io.Serializable);
import com.qualcomm.robotcore.hardware.ServoImpl;  // (implements import com.qualcomm.robotcore.hardware.Servo);
import com.qualcomm.robotcore.hardware.ServoImplEx;  // (implements import com.qualcomm.robotcore.hardware.PwmControl);
import org.firstinspires.ftc.robotcore.external.SignificantMotionDetection;
import com.qualcomm.ftccommon.SoundPlayer;  // (implements android.media.SoundPool.OnLoadCompleteListener);
//import com.qualcomm.ftccommon.SoundPlayer.CurrentlyPlaying;
//import com.qualcomm.ftccommon.SoundPlayer.InstanceHolder;
//import com.qualcomm.ftccommon.SoundPlayer.LoadedSoundCache;
import com.qualcomm.ftccommon.SoundPlayer.PlaySoundParams;
import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.StateTransition;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

//ThemedActivity

//import com.qualcomm.ftccommon.ConfigWifiDirectActivity;

//ThemedActivity

import com.qualcomm.ftccommon.FtcAboutActivity;

//ThemedActivity

import com.qualcomm.ftccommon.FtcAdvancedRCSettingsActivity;

//ThemedActivity

import com.qualcomm.ftccommon.FtcLynxFirmwareUpdateActivity;

//ThemedActivity

import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity;

//ThemedActivity

import com.qualcomm.ftccommon.FtcWifiDirectChannelSelectorActivity;  // (implements android.widget.AdapterView.OnItemClickListener);

//ThemedActivity

import com.qualcomm.ftccommon.FtcWifiDirectRememberedGroupsActivity;

//ThemedActivity

import com.qualcomm.ftccommon.ViewLogsActivity;
import java.lang.Throwable;  //(implements java.io.Serializable);
import java.lang.Exception;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.exception.RobotProtocolException;
import java.lang.RuntimeException;
import com.qualcomm.robotcore.exception.DuplicateNameException;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.TypeConversion;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftccommon.UpdateUI.Callback;
//import com.qualcomm.ftccommon.UpdateUI.Callback.DeviceNameManagerCallback;
//import com.qualcomm.ftccommon.USBAccessibleLynxModule;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRelicRecovery;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase.TrackingResults;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRelicRecovery.TrackingResults;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;  // (implements import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable.Listener);
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener.PoseAndCamera;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;

//Interface Hierarchy

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import org.firstinspires.ftc.robotcore.external.Consumer;  //<T>;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.Engagable;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple);
import org.firstinspires.ftc.robotcore.external.Event;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.EventLoopManager.EventLoopMonitor;
import com.qualcomm.robotcore.eventloop.opmode.EventLoopManagerClient;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import org.firstinspires.ftc.robotcore.external.Func;  //<T>;
import org.firstinspires.ftc.robotcore.external.Function;  //<T,R>;
//import com.qualcomm.robotcore.hardware.Gamepad.GamepadCallback;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;  // (also extends import com.qualcomm.robotcore.hardware.OrientationSensor);
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInputController;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;  // (also extends import com.qualcomm.robotcore.hardware.AnalogOutputController, import com.qualcomm.robotcore.hardware.DigitalChannelController, import com.qualcomm.robotcore.hardware.I2cController, import com.qualcomm.robotcore.hardware.PWMOutputController);
//import com.qualcomm.robotcore.hardware.AnalogOutputController;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;  // (also extends import com.qualcomm.robotcore.hardware.AnalogInputController, import com.qualcomm.robotcore.hardware.DigitalChannelController, import com.qualcomm.robotcore.hardware.I2cController, import com.qualcomm.robotcore.hardware.PWMOutputController);
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;  // (also extends import com.qualcomm.robotcore.hardware.AnalogInputController, import com.qualcomm.robotcore.hardware.AnalogOutputController, import com.qualcomm.robotcore.hardware.DigitalChannelController, import com.qualcomm.robotcore.hardware.I2cController, import com.qualcomm.robotcore.hardware.PWMOutputController);
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;  // (also extends import com.qualcomm.robotcore.hardware.AnalogInputController, import com.qualcomm.robotcore.hardware.AnalogOutputController, import com.qualcomm.robotcore.hardware.I2cController, import com.qualcomm.robotcore.hardware.PWMOutputController);
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.hardware.I2cController;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;  // (also extends import com.qualcomm.robotcore.hardware.AnalogInputController, import com.qualcomm.robotcore.hardware.AnalogOutputController, import com.qualcomm.robotcore.hardware.DigitalChannelController, import com.qualcomm.robotcore.hardware.PWMOutputController);
//import com.qualcomm.robotcore.hardware.LegacyModule;  // (also extends import com.qualcomm.robotcore.hardware.HardwareDevice);
import com.qualcomm.robotcore.hardware.I2cDevice;  // (also extends import com.qualcomm.robotcore.hardware.I2cControllerPortDevice);
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable, import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;  // (also extends import com.qualcomm.robotcore.hardware.HardwareDeviceHealth, import com.qualcomm.robotcore.hardware.I2cAddrConfig, import com.qualcomm.robotcore.hardware.RobotConfigNameable);
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable);
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
//import com.qualcomm.robotcore.hardware.LegacyModule;  // (also extends import com.qualcomm.robotcore.hardware.I2cController);
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;  // (also extends import com.qualcomm.robotcore.hardware.AnalogInputController, import com.qualcomm.robotcore.hardware.AnalogOutputController, import com.qualcomm.robotcore.hardware.DigitalChannelController, import com.qualcomm.robotcore.hardware.I2cController);
import com.qualcomm.robotcore.hardware.RobotConfigNameable;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable, import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;  // (also extends import com.qualcomm.robotcore.hardware.HardwareDevice, import com.qualcomm.robotcore.hardware.HardwareDeviceHealth, import com.qualcomm.robotcore.hardware.I2cAddrConfig);
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable);
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable, import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;  // (also extends import com.qualcomm.robotcore.hardware.HardwareDevice, import com.qualcomm.robotcore.hardware.I2cAddrConfig, import com.qualcomm.robotcore.hardware.RobotConfigNameable);
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable);
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable, import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;  // (also extends import com.qualcomm.robotcore.hardware.HardwareDevice, import com.qualcomm.robotcore.hardware.HardwareDeviceHealth, import com.qualcomm.robotcore.hardware.RobotConfigNameable);
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable);
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable, import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;  // (also extends import com.qualcomm.robotcore.hardware.HardwareDevice, import com.qualcomm.robotcore.hardware.HardwareDeviceHealth, import com.qualcomm.robotcore.hardware.I2cAddrConfig, import com.qualcomm.robotcore.hardware.RobotConfigNameable);
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;  // (also extends import com.qualcomm.robotcore.hardware.Engagable);
//import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyBeginEndNotifications;
//import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyCallback;
//import com.qualcomm.robotcore.hardware.I2cControllerPortDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;  // (also extends import com.qualcomm.robotcore.hardware.HardwareDevice);
import com.qualcomm.robotcore.hardware.I2cDeviceSynchReadHistory;
import java.lang.Iterable;  //<T>;
import java.util.Collection;  //<E>;
import java.util.List;  //<E>;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import com.qualcomm.robotcore.hardware.LegacyModulePortDevice;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection.MotionDetectionListener;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;  // (also extends import com.qualcomm.robotcore.hardware.Gyroscope);
import org.firstinspires.ftc.robotcore.external.Predicate;  //<T>;
//import com.qualcomm.ftccommon.ProgrammingModeController;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.PWMOutputControllerEx;
import com.qualcomm.robotcore.hardware.PWMOutputEx;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.robotcore.hardware.RobotCoreLynxModule;
import com.qualcomm.robotcore.hardware.RobotCoreLynxUsbDevice;
import org.firstinspires.ftc.robotcore.external.SignificantMotionDetection.SignificantMotionDetectionListener;
//import com.qualcomm.ftccommon.SoundPlayer.SoundFromFile;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Supplier;  //<T>;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.eventloop.SyncdDevice.Manager;
import com.qualcomm.robotcore.eventloop.SyncdDevice.Syncable;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry.Item;
import org.firstinspires.ftc.robotcore.external.Telemetry.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry.Log;
import org.firstinspires.ftc.robotcore.external.ThrowingCallable;  //<VALUE,EXCEPTION>;
import com.qualcomm.ftccommon.UsbModuleAttachmentHandler;
import com.qualcomm.robotcore.hardware.VisuallyIdentifiableHardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable.Listener;

//Annotation Type Hierarchy

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;  // (implements java.lang.annotation.Annotation);
import com.qualcomm.robotcore.eventloop.opmode.Disabled;  // (implements java.lang.annotation.Annotation);
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;  // (implements java.lang.annotation.Annotation);
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;  // (implements java.lang.annotation.Annotation);
import org.firstinspires.ftc.robotcore.external.Const;  // (implements java.lang.annotation.Annotation);
import org.firstinspires.ftc.robotcore.external.NonConst;  // (implements java.lang.annotation.Annotation);

//Enum Hierarchy

import java.lang.Object;
import java.lang.Enum;  //<E> (implements java.lang.Comparable<T>, java.io.Serializable);
//import com.qualcomm.ftccommon.ConfigWifiDirectActivity.Flag;
//import com.qualcomm.ftccommon.FtcLynxFirmwareUpdateActivity.FwResponseStatus;
import com.qualcomm.ftccommon.LaunchActivityConstantsList.RequestCode;
//import com.qualcomm.ftccommon.SoundPlayer.StopWhat;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import com.qualcomm.robotcore.eventloop.SyncdDevice.ShutdownReason;
import com.qualcomm.robotcore.hardware.CompassSensor.CompassMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DeviceManager.UsbDeviceType;
import com.qualcomm.robotcore.hardware.DigitalChannel.Mode;
//import com.qualcomm.robotcore.hardware.DigitalChannelController.Mode;
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth.HealthStatus;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl.READ_CACHE_STATUS;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl.WRITE_CACHE_STATUS;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl.CONTROLLER_PORT_MODE;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
//import com.qualcomm.robotcore.hardware.IrSeekerSensor.Mode;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
//import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoController.PwmStatus;
import org.firstinspires.ftc.robotcore.external.JavaUtil.AtMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil.TrimMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil.SortType;
import org.firstinspires.ftc.robotcore.external.JavaUtil.SortDirection;
import org.firstinspires.ftc.robotcore.external.Telemetry.Log.DisplayOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation.AngleSet;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters.CameraMonitorFeedback;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId.Type;

// import java.util.array;

// any improt that starts with "com.qualcom.robotcore" the link to find it is https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/package-summary.html
@TeleOp(name="trying", group="Linear Opmode")

public class Trying extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    public ElapsedTime runtime = new ElapsedTime();
    //public ElapsedTime time;
    public DcMotor LFD;
    public DcMotor LBD;
    public DcMotor RFD;
    public DcMotor RBD;
  
    public DcMotor armR;
    public Servo claw;
    public Servo claw2;
    
    private ColorSensor color;
    private DigitalChannel touch;
    
    public final static double CLAW_HOME = 0.1;     // Starting position for claw 
    public final static double CLAW_MIN_RANGE = 0.15;  // Smallest number value allowed for claw position
    public final static double CLAW_MAX_RANGE = 0.8;  // Largestnumber value allowed for claw position
    
    double clawPosition = CLAW_HOME;  // Sets safe position
    final double CLAW_SPEED = 0.7 ;  // Sets rate to move servo
    
    public final static double CLAW2_HOME = 0.6;     // Starting position for claw 
    public final static double CLAW2_MIN_RANGE = 0.10;  // Smallest number value allowed for claw position
    public final static double CLAW2_MAX_RANGE = 0.7;  // Largestnumber value allowed for claw position
    
    double claw2Position = CLAW2_HOME;  // Sets safe position
    final double CLAW2_SPEED = 1 ;  // Sets rate to move servo
    
    public void RunToPosition() {
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ArmRunToPosition() {
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void DriveRunToPosition() {
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void DriveResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetArmEncoder() {
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    // Send calculated power to motors
    public void Power(double Power) {
        //turret.setPower(turretPower);
        //arm.setPower(Power);
        LFD.setPower(Power);
        RFD.setPower(Power);
        LBD.setPower(Power);
        RBD.setPower(Power);
    }
    // public void Arm(int Pos1, double Power) {
    //     armR.setTargetPosition(Pos1);
    //     armR.setPower(Power);
        
    // }
    public void Arm(int Pos5) {
        armR.setTargetPosition(Pos5);
    }
    public void TelemetryN() {
        while (armR.isBusy() && LFD.isBusy() && RFD.isBusy() && LBD.isBusy() && RBD.isBusy()) {
            //telemetry.addData("EE, UU");
            telemetry.addData("Arm Encoder", armR.getCurrentPosition());
            telemetry.addData("Arm Power", "%.2f", armR.getPower());
            telemetry.update();
        }
    }
    public void TelemetryL() {
        while (armR.isBusy() && LFD.isBusy() && RFD.isBusy() && LBD.isBusy() && RBD.isBusy()) {
            telemetry.addData("Stauts", "UR MUM");
            telemetry.update();
        }
    }
    public void HoldPos(double Power) {
        armR.setPower(Power);
    }
    public void ArmPower(double Power) {
        armR.setPower(Power);
    }
    // public void ArmRuntoPsoition() {
    //     armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // }
    public void ArmVelocity(double Power) {
        ((DcMotorEx) armR).setVelocity(Power);
    }
    /*public void TelemetryM() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Medium Junction");
            telemetry.update();
        }
    }
    public void TelemetryHi() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "High Junction");
            telemetry.update();
        }
    }
    public void TelemetryH() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Home");
            telemetry.update();
        }
    }*/
    
    public void StopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void RunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void ArmRunUsingEncoders() {
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void ArmStopUsingEncoders() {
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void DriveRunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveStopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void Touch() {
            if (touch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                // ResetArmEncoder();
                //Arm(-1);
                ArmPower(-1);
                //ArmVelocity(100);
                //ArmRuntoPsoition();
                //Telemetry();
                
                
                telemetry.addData("Digital Touch", "Is Pressed");
            }
            }
    @Override
    public void runOpMode() {
        
        touch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        color = hardwareMap.get(ColorSensor.class, "color");
        LFD  = hardwareMap.get(DcMotor.class, "left front drive");
        LBD  = hardwareMap.get(DcMotor.class, "left back drive");
        RFD = hardwareMap.get(DcMotor.class, "right front drive");
        RBD = hardwareMap.get(DcMotor.class, "right back drive");
        //turret = hardwareMap.get(DcMotor.class, "turret");
        armR = hardwareMap.get(DcMotor.class, "arm right");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        LFD.setDirection(DcMotor.Direction.REVERSE);
        LBD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.FORWARD);
        RBD.setDirection(DcMotor.Direction.FORWARD);
        touch.setMode(DigitalChannel.Mode.INPUT);
        //turret.setDirection(DcMotor.Direction.FORWARD);
        
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        claw.setPosition(CLAW_HOME);
        claw2.setPosition(CLAW2_HOME);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        
        ResetEncoders();
        
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.x) {
                clawPosition += CLAW_SPEED;
                claw2Position -= CLAW2_MIN_RANGE;
            }
            else if (gamepad2.a) {
                clawPosition -= CLAW_SPEED;
                claw2Position += CLAW2_MAX_RANGE;
            }
            
            clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
            claw.setPosition(clawPosition);
            
            claw2Position = Range.clip(claw2Position, CLAW2_MIN_RANGE, CLAW2_MAX_RANGE);
            claw2.setPosition(claw2Position);
            
            // Display the current value
            
            // if (gamepad2.left_bumper) {
            //     ArmRunUsingEncoders();
            //     Arm(180);
            //     ArmPower(1);
            //     ArmRunToPosition();
            //     // HoldPos(0.05);
            //     TelemetryL();
            //     sleep(10);
            // // } else if (gamepad2.left_bumper = false) {
            // //     // double armPower;
            // // armR.setPower(armPower);
            // //      //ArmStopUsingEncoders();
            // // HoldPos(0);
            // // armPower = (1 * gamepad2.left_stick_y);
            // // Touch();
            // // armPower = (1 * -gamepad2.left_stick_y);
            // // // HoldPos(0.2);
            // // Touch();
            // // Telemetry();
            
            // }
            /*telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();*/

            idle();
            StopUsingEncoders();
            double max;
            
           
            
            double axial   =  -gamepad1.left_stick_y; // forward and backward
            double lateral = (0.7 * -gamepad1.right_stick_x); // turning left and right
            double yaw     =  -gamepad1.left_stick_x; // stafing
            //double turns = gamepad1.right_stick_x;
            //double turnss = -gamepad1.right_stick_x;
            
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            
            double RFPower = axial + lateral + yaw;
            double LFPower = axial - lateral - yaw;
            double LBPower = axial - lateral + yaw;
            double RBPower = axial + lateral - yaw;
            
            
            LFD.setPower(LFPower);
            RFD.setPower(RFPower);
            LBD.setPower(LBPower);
            RBD.setPower(RBPower);
            
            
            //RFPower = Range.clip(turns, -0.5, 0.5) ; 
            //RBPower = Range.clip(turnss, -0.5, 0.5) ; 
            //LFPower = Range.clip(turnss, 0.5, -0.5) ;
            //LBPower = Range.clip(turns, 0.5, -0.5) ;
            
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            /*max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
            max = Math.max(max, Math.abs(LFPower));
            max = Math.max(max, Math.abs(RFPower));
            max = Math.max(max, Math.abs(LBPower));
            max = Math.max(max, Math.abs(RBPower));

            if (max > 1.0) {
                LFPower = max;
                RFPower = max;
                LBPower = max;
                RBPower = max;
            }*/
            
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
            
            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
            //double turretPower;
            double armPower;
            
            //double drive = gamepad2.right_trigger;
            //double turn  =  -gamepad2.left_stick_y;
            //turretPower    = Range.clip(drive, -1.0, 1.0) ;
            //armPower   = Range.clip(turn, -1, 1);
            //armPower   = Range.clip(drive, -0.7, 0.7);
            
            // if (gamepad2.left_bumper == true) {
            //     ArmRunUsingEncoders();
            //     Arm(180);
            //     ArmPower(1);
            //     ArmRunToPosition();
            //     // HoldPos(0.05);
            //     TelemetryN();
            //     sleep(10);
            // } else if (gamepad2.left_bumper == false) {
                
            // } /*else if (gamepad2.left_bumper) {
            //     RunUsingEncoders();
            //     HoldPos(0);
            //     Arm(2900, 1);
            //     RunToPosition();
            //     Power(0);
            //     TelemetryL();
            //     HoldPos(0.1);
            //     TelemetryL();
            // } else if (gamepad2.b) {
            //     RunUsingEncoders();
            //     HoldPos(0);
            //     Arm(0, 1);
            //     RunToPosition();
            //     Power(0);
            //     TelemetryL();
            //     HoldPos(0.1);
            //     TelemetryL();
            // }*/
            
            ArmStopUsingEncoders();
            HoldPos(0);
            armPower = (1 * gamepad2.left_stick_y);
            Touch();
            armPower = (1 * -gamepad2.left_stick_y);
            HoldPos(0.2);
            Touch();
            //Telemetry();
            
            
            armR.setPower(armPower);
            
            
            /*LFPosition && RFPosition && LBPosition && RBPosition = (1 * -gamepad2.left_stick_y);
            
            LFPosition && RFPosition && LBPosition && RBPosition = (1 * -gamepad2.right_stick_x);*/
            
            
            /*LFD.setTargetPosition(LFPosition);
            RFD.setTargetPosition(RFPosition);
            LBD.setTargetPosition(LBPosition);
            RBD.setTargetPosition(RBPosition);
            arm.setTargetPosition(armPosition);*/
            
            

            //double time;
            //Telemetry();
            // Show the elapsed game time and wheel power.
            
            
                telemetry.addData("LF Encoder:", LFD.getCurrentPosition());
                telemetry.addData("RF Encoder:", RFD.getCurrentPosition());
                telemetry.addData("LB Encoder:", LBD.getCurrentPosition());
                telemetry.addData("RB Encoder:", RBD.getCurrentPosition());
                telemetry.addData("Arm Encoder", armR.getCurrentPosition());
                /*telemetry.addData("Left/Right", "%4.2f",  "%4.2f", LFPower, RFPower);
                telemetry.addData("Left/Right", "%4.2f",  "%4.2f", LBPower, RBPower);*/
                telemetry.addData("LF Power ", "%.2f", LFPower);
                telemetry.addData("RF Power", "%.2f", RFPower);
                telemetry.addData("LB Power", "%.2f", LBPower);
                telemetry.addData("RB Power", "%.2f", RBPower);
                telemetry.addData("Arm Power", "%.2f", armPower);
                telemetry.addData("Status", "Run Time: " + runtime.time(TimeUnit.MINUTES));
                //telemetry.addData("Color", "%1.0f", color.argb());
                //telemetry.addData("color", color.getrgba);
                telemetry.update();
                /*telemetry.addData("LF-Encoder/Power", "%4.2f", LFD.getCurrentPosition(), LFPower);
                telemetry.addData("RF-Encoder/Power", "%4.2f", RFD.getCurrentPosition(), RFPower);
                telemetry.addData("LB-Encoder/Power", "%4.2f", LBD.getCurrentPosition(), LBPower);
                telemetry.addData("RB-Encoder/Power", "%4.2f", RBD.getCurrentPosition(), RBPower);
                telemetry.addData("Arm-Encoder/Power", "%4.2f", armR.getCurrentPosition(), armPower);
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();*/
                /*telemetry.addData("Front left/Right", "%4.2f, %4.2f", LFPower, RFPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", LBPower, RBPower);
                telemetry.addData("Motors", "arm (%.2f)", armPower);
                telemetry.update();*/
        }
    }}


