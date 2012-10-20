
(in-package :asdf)

(defsystem "clearpath_horizon-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "DifferentialOutput" :depends-on ("_package"))
    (:file "_package_DifferentialOutput" :depends-on ("_package"))
    (:file "Distance" :depends-on ("_package"))
    (:file "_package_Distance" :depends-on ("_package"))
    (:file "PlatformName" :depends-on ("_package"))
    (:file "_package_PlatformName" :depends-on ("_package"))
    (:file "GPIO" :depends-on ("_package"))
    (:file "_package_GPIO" :depends-on ("_package"))
    (:file "RawEncoders" :depends-on ("_package"))
    (:file "_package_RawEncoders" :depends-on ("_package"))
    (:file "DifferentialSpeed" :depends-on ("_package"))
    (:file "_package_DifferentialSpeed" :depends-on ("_package"))
    (:file "PowerSource" :depends-on ("_package"))
    (:file "_package_PowerSource" :depends-on ("_package"))
    (:file "DistanceTiming" :depends-on ("_package"))
    (:file "_package_DistanceTiming" :depends-on ("_package"))
    (:file "VelocitySetpt" :depends-on ("_package"))
    (:file "_package_VelocitySetpt" :depends-on ("_package"))
    (:file "StateChange" :depends-on ("_package"))
    (:file "_package_StateChange" :depends-on ("_package"))
    (:file "Joy" :depends-on ("_package"))
    (:file "_package_Joy" :depends-on ("_package"))
    (:file "GPADCOutput" :depends-on ("_package"))
    (:file "_package_GPADCOutput" :depends-on ("_package"))
    (:file "FirmwareInfo" :depends-on ("_package"))
    (:file "_package_FirmwareInfo" :depends-on ("_package"))
    (:file "Encoder" :depends-on ("_package"))
    (:file "_package_Encoder" :depends-on ("_package"))
    (:file "AckermannSetpt" :depends-on ("_package"))
    (:file "_package_AckermannSetpt" :depends-on ("_package"))
    (:file "TurnSetpt" :depends-on ("_package"))
    (:file "_package_TurnSetpt" :depends-on ("_package"))
    (:file "PlatformInfo" :depends-on ("_package"))
    (:file "_package_PlatformInfo" :depends-on ("_package"))
    (:file "SafetyStatus" :depends-on ("_package"))
    (:file "_package_SafetyStatus" :depends-on ("_package"))
    (:file "RotateRate" :depends-on ("_package"))
    (:file "_package_RotateRate" :depends-on ("_package"))
    (:file "SystemStatus" :depends-on ("_package"))
    (:file "_package_SystemStatus" :depends-on ("_package"))
    (:file "Orientation" :depends-on ("_package"))
    (:file "_package_Orientation" :depends-on ("_package"))
    (:file "ProcessorStatus" :depends-on ("_package"))
    (:file "_package_ProcessorStatus" :depends-on ("_package"))
    (:file "PowerStatus" :depends-on ("_package"))
    (:file "_package_PowerStatus" :depends-on ("_package"))
    (:file "JoySwitch" :depends-on ("_package"))
    (:file "_package_JoySwitch" :depends-on ("_package"))
    (:file "Encoders" :depends-on ("_package"))
    (:file "_package_Encoders" :depends-on ("_package"))
    (:file "Announce" :depends-on ("_package"))
    (:file "_package_Announce" :depends-on ("_package"))
    (:file "DifferentialControl" :depends-on ("_package"))
    (:file "_package_DifferentialControl" :depends-on ("_package"))
    ))