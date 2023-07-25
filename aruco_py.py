#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

# <rtc-template block="description">
"""
 @file aruco_py.py
 @brief ModuleDescription
 @date $Date$


"""
# </rtc-template>

import sys, cv2
from cv2 import aruco
import numpy as np
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

isInImage = bool


# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
aruco_py_spec = ["implementation_id", "aruco_py", 
         "type_name",         "aruco_py", 
         "description",       "ModuleDescription", 
         "version",           "1.0.0", 
         "vendor",            "kawa", 
         "category",          "Category", 
         "activity_type",     "STATIC", 
         "max_instance",      "1", 
         "language",          "Python", 
         "lang_type",         "SCRIPT",
         ""]
# </rtc-template>

# <rtc-template block="component_description">
##
# @class aruco_py
# @brief ModuleDescription
# 
# 
# </rtc-template>
class aruco_py(OpenRTM_aist.DataFlowComponentBase):
	
    ##
    # @brief constructor
    # @param manager Maneger Object
    # 
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

        self._d_image = OpenRTM_aist.instantiateDataType(RTC.CameraImage)
        """
        """
        self._imageIn = OpenRTM_aist.InPort("image", self._d_image)
        self._d_6d = OpenRTM_aist.instantiateDataType(RTC.TimedDoubleSeq)
        """
        """
        self._6dOut = OpenRTM_aist.OutPort("6d", self._d_6d)

        self.isInImage = False


		


        # initialize of configuration-data.
        # <rtc-template block="init_conf_param">
		
        # </rtc-template>


		 
    ##
    #
    # The initialize action (on CREATED->ALIVE transition)
    # 
    # @return RTC::ReturnCode_t
    # 
    #
    def onInitialize(self):
        # Bind variables and configuration variable
		
        # Set InPort buffers
        self.addInPort("image",self._imageIn)
		
        # Set OutPort buffers
        self.addOutPort("6d",self._6dOut)
		
        # Set service provider to Ports
		
        # Set service consumers to Ports
		
        # Set CORBA Service Ports
        print("aruco is ready")
		
        return RTC.RTC_OK
	
    ###
    ## 
    ## The finalize action (on ALIVE->END transition)
    ## 
    ## @return RTC::ReturnCode_t
    #
    ## 
    #def onFinalize(self):
    #

    #    return RTC.RTC_OK
	
    ###
    ##
    ## The startup action when ExecutionContext startup
    ## 
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onStartup(self, ec_id):
    #
    #    return RTC.RTC_OK
	
    ###
    ##
    ## The shutdown action when ExecutionContext stop
    ##
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onShutdown(self, ec_id):
    #
    #    return RTC.RTC_OK
	
    ##
    #
    # The activated action (Active state entry action)
    #
    # @param ec_id target ExecutionContext Id
    # 
    # @return RTC::ReturnCode_t
    #
    #
    def onActivated(self, ec_id):
    
        return RTC.RTC_OK
	
    ##
    #
    # The deactivated action (Active state exit action)
    #
    # @param ec_id target ExecutionContext Id
    #
    # @return RTC::ReturnCode_t
    #
    #
    def onDeactivated(self, ec_id):
        cv2.destroyAllWindows()
    
        return RTC.RTC_OK
	
    ##
    #
    # The execution action that is invoked periodically
    #
    # @param ec_id target ExecutionContext Id
    #
    # @return RTC::ReturnCode_t
    #
    #
    def onExecute(self, ec_id):

        global isInImage

        # マーカーサイズ
        marker_length = 0.056 # [m]
        # マーカーの辞書選択
        # dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        camera_matrix = np.array([[2.24861379e+03, 0.00000000e+00, 2.01502882e+03],
                                    [0.00000000e+00, 2.25437686e+03, 1.28473568e+03],
                                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        distortion_coeff = np.array([[ 8.71390681e-02, -1.93136080e-01,  4.63674412e-04,  1.50404867e-04, 1.79991456e-01]])

        # 画像の受信
        if self._imageIn.isNew():
            # 画像読み込み
            image = self._imageIn.read()
            isInImage = True

            # 画像をファイル形式に整形
            frame = np.frombuffer(image.pixels, dtype=np.uint8)
            # frame = frame.reshape(image.height, image.width, 3)
            # print("要変更：画像サイズを決め打って整列しています")
            frame = frame.reshape(720, 1280, 3)
            # print( "rcv image data" )

        if isInImage == True:
            image = np.copy(frame)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(image, dictionary)
        
            # 可視化
            aruco.drawDetectedMarkers(image, corners, ids, (0,255,255))

            # 次ループのための処理
            isInImage = False

            if len(corners) > 0:
                # マーカーごとに処理
                for i, corner in enumerate(corners):
                    # rvec -> rotation vector, tvec -> translation vector
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, distortion_coeff)

                    # < rodoriguesからeuluerへの変換 >
                    # 不要なaxisを除去
                    tvec = np.squeeze(tvec)
                    rvec = np.squeeze(rvec)
                    # 回転ベクトルからrodoriguesへ変換
                    rvec_matrix = cv2.Rodrigues(rvec)
                    rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
                    # 並進ベクトルの転置
                    transpose_tvec = tvec[np.newaxis, :].T
                    # 合成
                    proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
                    # オイラー角への変換
                    euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]

                    print("ids : " + str(ids[i]))
                    print("x : " + str(tvec[0]))
                    print("y : " + str(tvec[1]))
                    print("z : " + str(tvec[2]))
                    print("roll : " + str(euler_angle[0]))
                    print("pitch: " + str(euler_angle[1]))
                    print("yaw  : " + str(euler_angle[2]))

                    # 送信データ形成
                    # -- ID --
                    self._d_6d.data.append(float(ids[i]))
                    # -- XYZ --
                    for i in range(3):
                        self._d_6d.data.append(tvec[i])
                    # -- RPY --
                    for i in range(3):
                        self._d_6d.data.append(euler_angle[i][0])

                    # 送信
                    self._6dOut.write()

                    # 次ループのためにクリア
                    self._d_6d.data.clear()

                    # 可視化
                    draw_pole_length = marker_length/2 # 現実での長さ[m]
                    aruco.drawAxis(image, camera_matrix, distortion_coeff, rvec, tvec, draw_pole_length)

                    print("send 6d information")
            cv2.imshow('drawDetectedMarkers', image)
            cv2.waitKey(10)

    
        return RTC.RTC_OK
	



def aruco_pyInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=aruco_py_spec)
    manager.registerFactory(profile,
                            aruco_py,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    aruco_pyInit(manager)

    # create instance_name option for createComponent()
    instance_name = [i for i in sys.argv if "--instance_name=" in i]
    if instance_name:
        args = instance_name[0].replace("--", "?")
    else:
        args = ""
  
    # Create a component
    comp = manager.createComponent("aruco_py" + args)

def main():
    # remove --instance_name= option
    argv = [i for i in sys.argv if not "--instance_name=" in i]
    # Initialize manager
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(MyModuleInit)
    mgr.activateManager()
    mgr.runManager()

if __name__ == "__main__":
    main()

