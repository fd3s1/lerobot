#!/usr/bin/env python3
# coding: utf-8
import rospy
#import stdio 
from std_msgs.msg import Float64
import numpy as np
from matplotlib import pyplot as plt
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CS16
import time
#from test_wifi.msg import uav_sgn_status
import signal
from quadrotor_msgs.msg import sgn_stamp

class PowerSense:

    def __init__(self, centerFreq, sampRate, N, gain, useAmpd, plotPicture):
        """
        初始化感知程序
        :param centerFreq: 感知中心频率
        :param sampRate: 采样率
        :param N: 每次感知收到的点数，若为多次感知则一共也为N
        :param gain: 接收增益
        :param useAmpd: 是否使用峰值检测
        :param plotPicture: 是否画图展示
        """
        self._useAmpd = useAmpd
        self._plotPicture = plotPicture
        # 存放需要感知的中心频率
        self._centerFreqList = [centerFreq]
        # 感知频率范围最左边的频率值
        startFreq = centerFreq - sampRate / 2
        # 如果感知带宽大于40兆则分多次感知
        senseNum = 1
        if sampRate > 40e6:
            while sampRate > 40e6:
                sampRate /= 2
                senseNum *= 2
            self._centerFreqList = []
            for i in range(senseNum):
                if i == 0:
                    self._centerFreqList.append(startFreq + sampRate / 2)
                else:
                    self._centerFreqList.append(self._centerFreqList[-1] + sampRate)

        self.__init_usrp(sampRate, gain)
        self.N = int(N / senseNum)
        # 划分频点，方便绘图使用
        self._freqList = startFreq + np.arange(0, sampRate * senseNum, sampRate * senseNum / N)

    def __init_usrp(self, sampRate, gain):
        """初始化usrp设备"""
        # 端口编号 RX1 = 0, RX2 = 1
        self._rxChan = 0
        # 定义设备 N210:driver="uhd", type="usrp2"  AirT:driver="SoapyAIRT"  B210:type="b200"
        self._sdr = SoapySDR.Device(dict(driver="uhd", type="b200"))
        self._sdr.setSampleRate(SOAPY_SDR_RX, 0, sampRate)
        self._sdr.setGain(SOAPY_SDR_RX, 0, gain)

    def __get_iqdata(self, senseCenterFreq):
        """
        获取IQ数据
        :param senseCenterFreq:感知中心频率
        :return:存放iq数据的复数列表
        """
        # 每次都设置数据流并激活设备，不然会有残留数据，可能有别的方法清空数据
        rxStream = self._sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [self._rxChan])
        # 这里在采样前先切换频点 保证在相应的区域扫频
        self._sdr.setFrequency(SOAPY_SDR_RX, 0, senseCenterFreq)
        self._sdr.activateStream(rxStream)
        rxBuff = np.empty(2 * self.N, np.int16)
        sr = self._sdr.readStream(rxStream, [rxBuff], self.N, timeoutUs=int(5e6))
        # 读取得到的数据点数,如果获得数据与N不相等 等待下次获取
        while sr.ret != self.N:
            # rxBuff = np.empty(2 * self.N, np.int16)
            sr = self._sdr.readStream(rxStream, [rxBuff], self.N, timeoutUs=int(5e6))

        s0 = rxBuff.astype(float) / np.power(2.0, 12 - 1)  # IQ数据存于此处
        # 提取合并成复数IQ数据
        iqData = (s0[::2] + 1j * s0[1::2])
        self._sdr.deactivateStream(rxStream)
        self._sdr.closeStream(rxStream)
        return iqData

    def __get_power(self, iqData):
        """
        通过FFT获取每个采样点的功率
        :param iqData: 存放iq数据的复数列表
        :return: 存放功率值的数组
        """
        fftData = np.fft.fftshift(np.fft.fft(iqData, self.N) / self.N)
        power = 20 * np.log10(np.abs(fftData))
        return power

    def __get_peakindex(self, data):
        """
        得到给定数组的所有峰值下标
        :param data: 需要求峰值位置的数组
        :return:存放峰值所处下标的数组
        """
        # top分别为0，1，2就可以代表第一高（波峰），第二高、第三高等。
        top = 0
        pData = np.zeros_like(data, dtype=np.int32)
        count = data.shape[0]
        rowSumArr = []
        # 找到极大值位置数量最多的时候窗口大小为多少
        for k in range(1, count // 2 + 1):
            rowSum = 0
            for i in range(k, count - k):
                if data[i] > data[i - k] and data[i] > data[i + k]:
                    rowSum -= 1
            rowSumArr.append(rowSum)
        maxCountWindowLength = np.argmin(rowSumArr) + 1
        # 求出每一位置在窗口大小为1到maxCountWindowLength的极大值点数量
        for k in range(1, maxCountWindowLength + 1):
            for i in range(k, count - k):
                if data[i] > data[i - k] and data[i] > data[i + k]:
                    pData[i] += 1
        # 返回极大值点数量等于窗口大小的数据的位置，即它在1到maxCountWindowLength每一个窗口大小都属于是极值点
        return np.where(pData == maxCountWindowLength - top)[0]

    def __plot_powergraph(self, power, time, peakPowerIndex):
        """
        绘制功率图
        :param power:存放功率的数组
        :param time: 图像刷新时间
        :param peakPowerIndex: 存放峰值所在下标数组
        """
        # overAveragePower = np.average(pw) + 15
        # supportLine = np.zeros(len(pw)).tolist()
        # for i in range(len(pw)):
        #     supportLine[i] = overAveragePower
        plt.clf()
        plt.plot(self._freqList, power)
        if len(peakPowerIndex) != 0:
            peakPower = [self._freqList[i] for i in peakPowerIndex]
            plt.scatter(peakPower, power[peakPowerIndex], color="red")
        # plt.plot(self.f_ghz, supportLine, color='r', linestyle="--", linewidth=2)
        plt.xlim(self._freqList[0], self._freqList[-1])
        plt.ylim(-80, 20)
        plt.pause(time)

    def get_averagepower_and_iqdata(self):
        """得到感知范围的平均功率大小，当使用了AMPD则返回峰值的平均功率，否则返回所有采样点的平均功率"""
        peakPowerIndex = np.array([])
        peakPower = np.array([])
        allPower = np.array([])
        #allIqData = np.array([])
        startTime = time.time()
        for i in range(len(self._centerFreqList)):
            iqData = self.__get_iqdata(self._centerFreqList[i])
            #allIqData = np.append(allIqData,iqData)
            curPower = self.__get_power(iqData)
            allPower = np.append(allPower, curPower)
        if self._useAmpd:
            peakPowerIndex = self.__get_peakindex(allPower)
            peakPower = allPower[peakPowerIndex]
            peakPower = np.sort(peakPower)[::-1][:5]
        if self._plotPicture:
            self.__plot_powergraph(allPower, time.time() - startTime, peakPowerIndex)
        allPower = np.sort(allPower)[::-1][:5]
        return float(np.average(allPower)) if len(peakPower) == 0 else float(np.average(peakPower))
    
    def mySigintHandler(sig):
        rospy.signal_shutdown()



if __name__ == '__main__':
    rospy.init_node('usrp_001_node')
    pub=rospy.Publisher('/usrp_power_001',sgn_stamp,queue_size = 10)
    rate=rospy.Rate(50)
    signal.signal(signal.SIGINT, PowerSense.mySigintHandler)
    
    try:
        ps = PowerSense(2.402e9, 10e6, 1024, 20, False, False)
        file = open("senseData", 'w')
        while True:
            startTime = time.time()
            power =ps.get_averagepower_and_iqdata()
            #print(power)
            uav_status=sgn_stamp()
            uav_status.header.stamp=rospy.get_rostime()
            #uav1_usrp1=Float64()
            uav_status.rssi=float(power)
            pub.publish(uav_status)
            rate.sleep()
            #print(uav1_usrp1)
            #print(iqdata)
            #file.write(time.asctime())
            #file.write(":power-->")
            #file.write(str(power)+"\n")
            #file.write(":iqdata-->")
            #file.write(str(iqdata)+"\n")
            #print(time.time() - startTime)
        rospy.spin()
        file.close()
    except KeyboardInterrupt:
        pass

	
    # filename = r"Client"
    # data = []
    # try:
    #     # 打开文件
    #     fp = open(filename, "r")
    #     for line in fp.readlines():
    #         '''
    #         每行数据末尾都会有换行符‘\n’，所以我们需要先把它去掉
    #         '''
    #         line = line.replace('\n', '')
    #         data.append(line)
    #     fp.close()
    # except IOError:
    #     print("文件打开失败，%s文件不存在" % filename)
