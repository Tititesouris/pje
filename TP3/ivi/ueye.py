"""
.. module:: ueye
    :synopsis: Python wrapper module for IDS uEye Cameras. Based on pydcu project by wmpauli, https://github.com/wmpauli/pydcu that uses ueye_api DLL.

.. moduleauthor:: Benjamin Mathon and Olivier Losson
"""

 # This file is part of pydcu.

 #    pydcu is free software: you can redistribute it and/or modify
 #    it under the terms of the GNU General Public License as published by
 #    the Free Software Foundation, either version 3 of the License, or
 #    (at your option) any later version.

 #    pydcu is distributed in the hope that it will be useful,
 #    but WITHOUT ANY WARRANTY; without even the implied warranty of
 #    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 #    GNU General Public License for more details.

 #    You should have received a copy of the GNU General Public License
 #    along with pydcu.  If not, see <http://www.gnu.org/licenses/>.

import ctypes
import logging
import os
import os.path
import sys
import time
from ctypes import byref,c_int,create_string_buffer, c_char_p,cdll,util,c_void_p
from os.path import expanduser # to determine user home directory

import numpy as np

import IS

SUCCESS = 0
NO_SUCCESS = -1
INVALID_HANDLER = 1
HANDLE = c_void_p            
HCAM = HANDLE
HWND = c_void_p            
INT = c_int
UINT = ctypes.c_uint
DOUBLE = ctypes.c_double

if os.name=='nt':
    libname = 'ueye_api_64'
    include_ueye_h = os.environ['PROGRAMFILES']+'\\image\\uEye\\Develop\\Include\\ueye.h'
if os.name=='posix':
    libname = 'ueye_api'
    include_ueye_h = "/usr/include/ueye.h"
lib = util.find_library(libname)
if lib is None:
    print 'ueye.dll not found'
        
libueye= cdll.LoadLibrary(lib)
logger = logging.getLogger()

def CALL(name, *args):
    # Calls libueye function "name" and arguments "args".
    funcname = 'is_' + name
    func = getattr(libueye, funcname)
    new_args = []
    for a in args:        
        if isinstance (a, unicode):
            print name, 'argument',a, 'is unicode'
            new_args.append (str (a))
        else:
            new_args.append (a)
    return func(*new_args) 


class camera(HCAM):
    """
        An IDS uEye camera.
    """

    def __init__(self):
        c_void_p.__init__(self,0)
        r = CALL('InitCamera',byref(self),HWND(0))
        if r is not SUCCESS:
            raise Exception("Error %d: is the camera plugged in?"%r)
        self.width = 752
        self.height = 480
        self.seq = 0
        #self.data = zeros((self.height,self.width),dtype=np.uint8)
        #self.ctypes_data = (ctypes.c_int * (752 * ((8 + 1) / 8 + 0) * 480))()
        self.mem_buffer = ctypes.pythonapi.PyBuffer_FromMemory
        self.mem_buffer.restype = ctypes.py_object
        self.sizes = self.enum_sizes()
        self.sizes_menu = dict(zip([str(w)+"x"+str(h) for w,h in self.sizes], range(len(self.sizes))))
        self.rates = self.enum_rates()
        self.rates_menu = dict(zip([str(float(d)/n) for n,d in self.rates], range(len(self.rates))))
        fps = self.rates[2]
        try:
            self.current_rate_idx = self.rates.index(fps)
        except ValueError:
            logger.warning("Buggy Video Camera: Not all available rates are exposed.")
            self.current_rate_idx = 0

    def CheckForSuccessError(self,return_value):
        if return_value is not SUCCESS:
            self.GetError()
            raise Exception(self.error_message.value)
        return SUCCESS

    def CheckForNoSuccessError(self,return_value):
        if return_value is NO_SUCCESS:
            self.GetError()
            raise Exception(self.error_message.value)
        return return_value

    def AddToSequence(self):
        """
        Inserts image memory into the image memory list,
        which is to be used for ring buffering. The image memory has to
        be allocated with AllocImageMem(). All image memory which is 
        used for ring buffering must have been allocated the same colour
        depth (i.e. bits per pixel). The number of image memories for a
        sequence (nID) is limited to the integer value range.  
        """
        self.seq += 1
        r = CALL('AddToSequence',self,self.image,self.id) 
        return self.CheckForSuccessError(r)

    def ClearSequence(self):
        """
        Deletes all image memory from the sequence list
        that was inserted with AddToSequence(). After ClearSequence() no
        more image memory is active. To make a certain part of the image
        memory active, SetImageMem() and SetImageSize() have to be 
        executed.
        Not tested!
        """
        r = CALL('ClearSequence',self)
        return self.CheckForSuccessError(r)
            
    def LockSeqBuf(self,number):
        """
        Can be used to disable the overwriting of the image
        memory with new image data. And thus it is possible to prevent
        images which are required for further processing from being 
        overwritten. Full access to the image memory is still available.
        Only one image memory can be disabled at the same time. To 
        access the image memory use function UnlockSeqBuf().
        Not tested!
        """
        r = CALL('LockSeqBuf',self,INT(number),self.image)
        return self.CheckForSuccessError(r)

    def UnlockSeqBuf(self,number):
        """
        Image acquisition is allowed in a previously
        locked image memory. The image memory is put to the previous 
        position in the sequence list.
        """
        r = CALL('UnlockSeqBuf',self,INT(number),self.image)
        return self.CheckForSuccessError(r)

    def GetLastMemorySequence(self):
        """
        Returns the ID of the last
        recorded sequence in the memory board. This parameter can then 
        be used in combination with the function TransferImage() to read
        images out of the camera memory.
        No memory board to test this, Not tested!
        """
        r = CALL('GetLastMemorySequence',self,byref(self.id))
        return self.CheckForSuccessError(r)

    def TransferImage(self):
        """
        Experiment to find out how it works
        TransferImage(self, INT nMemID, INT seqID, INT imageNr, INT reserved)
        Not in the user manual!
        Not implemented!
        """
        CALL('TransferImage',self,INT(),INT(),INT(),INT())

    # def TransferMemorySequence():
    #     """
    #     Experiment to find out how it works
    #     TransferMemorySequence(HIDS hf, INT seqID, INT StartNr, INT nCount, INT nSeqPos);
    #     Not in the user manual!
    #     Not implemented!
    #     """
    #     pass

    def GetMemorySequenceWindow(self,id):
        """
        Checks the window size of a specified memory board sequence. The assigned
        sequence ID is required as a parameter.
        Not tested!
        """
        top    = INT() 
        left   = INT()
        right  = INT()
        bottom = INT()
        CALL('GetMemorySequenceWindow',self,INT(id),byref(left),byref(top),byref(right),byref(bottom))
        return (left.value,top.value,right.value,bottom.value)

    def GetActSeqBuf(self):
        """
        Gets the image memory in which image acquisition
        (ppcMem) is currently taking place and the image memory which 
        was last used for image acquisition (ppcMemLast).
        This function is only available when the ring buffer is
        active. If image acquisition is started for a ring buffer, 
        GetActSeqBuf returns 0 in pnNum as long as data is acquired to 
        the first image memory of the sequence. And thus pnNum receives
        the number of the sequence image memory, in which image acqui-
        sition is currently taking place. The number is not the ID of 
        the image memory which is provided from AllocImageMem(), but the
        running number in the sequence as defined in AddToSequence().
        """
        aqID = INT()
        pcMem = c_char_p()
        pcMemLast = c_char_p()
        paqID = byref(aqID)
        ppcMem = byref(pcMem)
        ppcMemLast = byref(pcMemLast)
        r = CALL('GetActSeqBuf',self,paqID,ppcMem,ppcMemLast)
        return self.CheckForSuccessError(r)
        
    def AllocImageMem(self, width=752, height=480, depth=8):
        """
        Allocates image memory for this camera.

        Memory size is at least:
        :math:`size = [width * ((depth + 1) / 8) + adjust] * height`.
        Line increments are calculated with:
        :math:`lineinc = line + adjust`,
        where:
        :math:`line = width * [(depth + 1) / 8]`, :math:`adjust  = 0` when :math:`line` without rest is divisible by 4, and :math:`adjust  = 4 - rest(line / 4)` otherwise.
        Before images can be digitized to this new memory location, it has to be activated with :func:`SetImageMem`,
        followed by :func:`SetImageSize` so that the image conditions can be transferred to the newly activated memory location.

        Freeing the memory is achieved with :func:`FreeImageMem`. In the DirectDraw modes, the allocation of an image memory is not required!

        :param width: image width
        :type width: int
        :param height: image height
        :type height: int
        :param depth: image depth in bits
        :type depth: int
        """
        self.image = c_char_p()
        #self.image = POINTER(c_char)()
        self.id = INT()
        r =  CALL('AllocImageMem', self,
                  INT(width),
                  INT(height),
                  INT(depth),
                  byref(self.image),
                  byref(self.id))
        return self.CheckForSuccessError(r)

    def GetNumberOfMemoryImages(self):
        """
        The function GetNumberOfMemoryImages() returns the number of 
        valid images that are currently located in the camera memory 
        within the specified sequence ID. This number can differ from 
        the originally recorded number of images because of overwriting.
        Not tested!

        """
        number = INT()
        r = CALL('GetNumberOfMemoryImages',self,INT(self.seq),byref(number))
        return self.CheckForSuccessError(r)
    
    def SetImageMem(self):
        """
        SetImageMem() sets the allocated image memory to active memory.
        Only an active image memory can receive image data. After 
        calling SetImageMem() function SetImageSize() must follow to set
        the image size of the active memory. A pointer from function
        AllocImgMem() has to be given to parameter pcImgMem.
        """
        r = CALL("SetImageMem",self,self.image,self.id)
        return self.CheckForSuccessError(r)

#    def CopyImageMem(self):
#        """
#        CopyImageMem() copies the contents of the image memory, as
#        described is pcSource and nID to the area in memory, which
#        pcDest points to.
#        """
#        r = CALL("CopyImageMem",self,self.image,self.id,byref(self.ctypes_data))
#        buffer = self.mem_buffer(self.ctypes_data, self.width * self.height)
#        print 'fwidth = ',self.width,', fheight = ',self.height
#        self.data = np.frombuffer(buffer, np.dtype('uint8'))
#        self.data = np.reshape(self.data, [self.height,self.width])
#
#        return self.CheckForSuccessError(r)

    def FreezeVideo(self, wait=IS.WAIT):
        """
        Acquires a single image from this camera (in a memory buffer).

        :param wait: Timeout value for image capture
        :type wait: integer
        """
        CALL("FreezeVideo",self,INT(wait))

    def toNumpyArray(self, iChannels):
        """
        Reads the buffered image (of type uint8) for this camera.

        :param iChannels: Number of channels of the image
        :type iChannels: integer

        :return: image data
        :rtype: 3-D numpy array
        """
        [width, height] = self.GetImageSize()
        buff = self.mem_buffer(self.image, width * height * iChannels)
        data = np.frombuffer(buff, np.dtype('uint8'))
        data = np.reshape(data, [height, width, iChannels])
        return data

        
    def SetImageSize(self, x=IS.GET_IMAGE_SIZE_X_MAX, y=IS.GET_IMAGE_SIZE_Y_MAX):#non-zero ret
        """
        Sets the image size for this camera.

        :param x: image width
        :type x: integer
        :param y: image height
        :type y: integer
        """
        r = CALL("SetImageSize",self,INT(x),INT(y))
        if x & IS.GET_IMAGE_SIZE_X == IS.GET_IMAGE_SIZE_X:
            return self.CheckForNoSuccessError(r)
        return self.CheckForSuccessError(r)

    def GetImageSize(self):
        """
        Gets the image size for this camera.

        :return: image dimensions [width, height]
        :rtype: 1-D numpy array
        """
        width_c=ctypes.c_int(IS.GET_IMAGE_SIZE_X)
        height_c=ctypes.c_int(IS.GET_IMAGE_SIZE_Y)
        width = CALL("SetImageSize",self, width_c)
        height = CALL("SetImageSize",self, height_c)
        #if r == 0:
        return ([width, height])

        #else:
        #    return self.CheckForSuccessError(r)
        

    def FreeImageMem(self):
        """
        Deallocates previously allocated image memory.
        For pcImgMem one of the pointers from AllocImgMem() has to be 
        used. All other pointers lead to an error message! The repeated
        handing over of the same pointers also leads to an error message
        """
        r = CALL("FreeImageMem",self,self.image,self.id)
        return self.CheckForSuccessError(r)

    def SetAllocatedImageMem(self,width=260,height=216,bitpixel=8):
        """
        Set an allocated memory, that was not allocated using 
        AllocImageMem, to the driver so it can be used to store the 
        image that will be digitized. The allocated memory must be
        globally locked.
        (Basically, use this if some non-driver function happens to have
        some memory already allocated, so you don't need to allocate more
        memory)
        Not Implemented!
        """
        self.image = self.data.ctypes.data_as(c_char_p)
        self.id = INT()
        r = CALL('SetAllocatedImageMem',self,
            INT(width),
            INT(height),
            INT(bitpixel),
            self.data.ctypes.data,
            byref(self.id))
        return self.CheckForSuccessError(r)
        
    def GetActiveImageMem(self):
        """
        Returns the pointer to the beginning and the
        ID number of the active memory. If DirectDraw mode is active and
        image memory has been allocated, this function returns the 
        pointer and the ID of the image memory, which was activated with
        SetImageMem(). However, it should be noted that in DirectDraw 
        mode, this memory is not used for digitizing.
        Also see GetImgMem().
        Not tested!
        """
        CALL('GetActiveImageMem',self,byref(self.image),byref(self.id))

    def GetImageMem(self):
        """
        Returns the pointer to the start of the active
        image memory. In DirectDraw mode the pointer is returned to the
        back buffer (or the visible area - DirectDraw Primary Surface
        mode).
        """
        CALL('GetImageMem',self,byref(self.image))

    def GetError(self):
        self.error = INT()
        self.error_message = c_char_p()
        return CALL("GetError",self,
            byref(self.error),
            byref(self.error_message))

    def SaveImage(self,filename):
        file = None if filename == None else c_char_p(filename)
        print file
        r = CALL('SaveImage',self,file)
        return self.CheckForSuccessError(r)

    def SaveImageEx(self, filename=None, format=IS.IMG_JPG, param=75):
        file = None if filename == None else c_char_p(filename)
        r = CALL('SaveImageEx',self,file,INT(format))#,INT(format),INT(param))
        return self.CheckForSuccessError(r)

    def SaveImageMemEx(self,file,type="jpg"):
        r = CALL('SaveImageMemEx',self,None)
        return self.CheckForSuccessError(r)

    def SetImagePos(self,x=0,y=0):
        """
        Sets the image position for this camera.

        :param x: image x-coordinate
        :type x: integer
        :param y: image y-coordinate
        :type y: integer
        """
        r = CALL("SetImagePos",self,INT(x),INT(y))
        if x & IS.GET_IMAGE_POS_X == IS.GET_IMAGE_POS_X:
            return self.CheckForNoSuccessError(r)
        return self.CheckForSuccessError(r)
        
    def CaptureVideo(self, wait=IS.DONT_WAIT):
        """
        Digitizes video images in real time and transfers the images to the previously allocated image memory.
        Alternatively if you are using DirectDraw the images can be 
        transferred to the graphics board. The image acquisition (DIB 
        Mode) takes place in the memory which has been set by 
        SetImageMem() and AllocImageMem(). GetImageMem() determines 
        exactly where the start address in memory is. In case of ring 
        buffering, then image acquisition loops endlessly through all 
        image memories added to the sequence.

        :param wait: wait time (x 10ms) until image acquisition begins after V-SYNC
        :type wait: integer

        Example::

            CaptureVideo(100)    # waits 1s, then starts video capture

        """
        r = CALL("CaptureVideo",self,INT(wait))
        return self.CheckForSuccessError(r)
        

    def LoadParameters(self):
        """ Load Parameter File saved previously with ueye demo """
        home_dir = expanduser("~")
        parameter_file = home_dir + '/UI154xLE-M_conf.ini'
        if os.path.isfile(parameter_file): 
            print "Loading Paramters for Camera"
            r = CALL("LoadParameters",self,c_char_p(parameter_file))
            print r
        else:
            print "File not Found: %s" % parameter_file
            return False
        return self.CheckForSuccessError(r)

    def SetColorMode(self, color_mode=IS.SET_CM_Y8):
        r = CALL("SetColorMode",self,INT(color_mode))
        return self.CheckForNoSuccessError(r)
    
    def SetSubSampling(self, mode=IS.SUBSAMPLING_DISABLE):
        r = CALL("SetSubSampling",self,INT(mode))
        return self.CheckForSuccessError(r)
        
    def StopLiveVideo(self, wait=IS.DONT_WAIT):
        """
        Freezes the image in the VGA card
        or in the PC's system memory. The function is controlled with 
        the parameter Wait. The function has two modes: Using the first
        mode, the function immediately returns to the calling function 
        and grabs the image in the background. In the second mode the 
        function waits until the image has been completely acquired and
        only then does the function return.
        By the use of IS.FORCE_VIDEO_STOP a single frame recording which
        is started with FreezeVideo(IS.DONT_WAIT) can be terminated
        immediately.
        """
        r = CALL("StopLiveVideo",self,INT(wait))
        return self.CheckForSuccessError(r)
        
    def ExitCamera (self):
        r = CALL("ExitCamera",self)
        return self.CheckForSuccessError(r)
    

    def ReadEEPROM(self,offset = 0, count = 64):
        """
        There is a rewritable EEPROM in the camera which serves as a 
        small memory. Additionally to the information which is stored 
        in the EEPROM, 64 extra bytes can be written. With the 
        ReadEEPROM() command the contents of these 64 bytes can be read.
        See WriteEEPROM.
        """
        if offset + count > 64:
            sys.stderr.write("offset + count too big, must be smaller or equal 64")
            raise
        buffer = create_string_buffer(count)
        CALL('ReadEEPROM',self,INT(offset),buffer,INT(count))
        return buffer.value

    def WriteEEPROM(self, content, offset = 0):
        """    
        In the DCU camera there is a rewritable EEPROM, where 64 bytes 
        of information can be written. With the ReadEEPROM() function
        the contents of this 64 byte block can be read.
        """
        count = len(content)
        if count + offset > 64:
            raise Exception("Content to long")
        pcString = c_char_p(content)
        r = CALL('WriteEEPROM',self,INT(offset),pcString,INT(count))
        return self.CheckForSuccessError(r)

    def Exposure(self, time):
        """
        Sets the exposure time (in ms).
        """
        IS_EXPOSURE_CMD_SET_EXPOSURE = 12 #there is a whole list to implement
        TIME = DOUBLE(time)
        nSizeOfParam = 8
        CALL('Exposure', self, 
                UINT(IS_EXPOSURE_CMD_SET_EXPOSURE), 
                byref(TIME), 
                UINT(nSizeOfParam))

    def InquireImageMem(self):
        """
        Reads the properties of the allocated image memory.
        """
        self.pnX = INT() 
        self.pnY = INT()
        self.pnBits  = INT()
        self.pnPitch = INT()
        CALL('InquireImageMem', self, self.image, self.id, byref(self.pnX), byref(self.pnY), byref(self.pnBits), byref(self.pnPitch)) 


    def GetColorDepth(self):
        """
        Gets the current VGA card colour setting and returns the bit depth (pnCol)
        and the related colour mode (pnColMode). The colour mode can be directl
        passed to the is_SetColorMode() function. 
        """
        self.pnCol = INT() 
        self.pnColMode = INT()
        r = CALL('GetColorDepth', self, byref(self.pnCol), byref(self.pnColMode))
        return self.CheckForSuccessError(r)


    def SetAOI(self, isType=IS.SET_IMAGE_AOI, x=0, y=0, width=752, height=480):
        """
        Sets the size, position, and type of image AOI (Area of Interest) for this camera.

        :param isType: AOI type (IS_SET_IMAGE_AOI | IS_SET_AUTO_BRIGHT_AOI | IS_SET_AUTO_WB_AOI)
        :type isType: integer
        :param x: horizontal position of AOI
        :type x: integer
        :param y: vertical position of AOI
        :type y: integer
        :param width: width of AOI
        :type width: integer
        :param height: height of AOI
        :type height: integer
        """
        logger.info("orig AOI vals: %s, %s, %s, %s" % (x, y, width, height))
        #x = int(x/4) * 4
        #y = int(y/4) * 4
        #width = int(width/4) * 4
        #height = int(height/4) * 4
        xPos_c = ctypes.c_int(x)
        yPos_c = ctypes.c_int(y)
        width_c = ctypes.c_int(width)
        height_c = ctypes.c_int(height)
        isType_c = ctypes.c_int(isType)
        logger.info("rounded AOI vals: %s, %s, %s, %s" % (x, y, width, height))
        r = CALL('SetAOI', self, isType_c, byref(xPos_c), byref(yPos_c), byref(width_c), byref(height_c))
        isType_c = ctypes.c_int(IS.SET_AUTO_BRIGHT_AOI)
        # r = CALL('SetAOI', self, isType_c, byref(xPos_c), byref(yPos_c), byref(width_c), byref(height_c))        
        # isType_c = ctypes.c_int(IS.SET_AUTO_WB_AOI)
        #r = CALL('SetAOI', self, isType_c, byref(xPos_c), byref(yPos_c), byref(width_c), byref(height_c))
        self.roi = [0, 0, height, width]
        self.t_roi = [y, x, y+height, x+width]
        if r == 0:
            return([xPos_c.value, yPos_c.value, width_c.value, height_c.value])
        else:
            return self.CheckForSuccessError(r)

    def GetAOI(self, isType=IS.GET_IMAGE_AOI, x=0, y=0, width=0, height=0):
        return self.SetAOI(isType, x, y, width, height)
    

    def SetAutoParameter(self, isType=IS.SET_ENABLE_AUTO_SHUTTER, pval1=1, pval2=0):
        '''
        Controls Auto Gain, Auto Shutter, Auto Framerate and Auto Whitebalance
        functionality. Purpose of the auto functions is it to control the camera 
        image in its average
        '''
        pval1_c = ctypes.c_double(pval1)
        pval2_c = ctypes.c_double(pval2)
        isType_c = ctypes.c_int(isType)
        r = CALL('SetAutoParameter', self, isType_c, byref(pval1_c), byref(pval2_c))
        pval1 = pval1_c.value
        pval2 = pval2_c.value
        ret = dict()
        ret['status'] = r
        ret['pval1'] = pval1
        ret['pval2'] = pval2
        return ret

    def enableAutoGain(self):
        r = self.SetAutoParameter(isType=IS.SET_ENABLE_AUTO_GAIN, pval1=1, pval2=0)
        return r

    def enableAutoExposure(self):
        r = self.SetAutoParameter(isType=IS.SET_ENABLE_AUTO_SHUTTER, pval1=1, pval2=0)
        return r

    def disableAutoGain(self):
        r = self.SetAutoParameter(isType=IS.SET_ENABLE_AUTO_GAIN, pval1=0, pval2=0)
        return r

    def disableAutoExposure(self):
        r = self.SetAutoParameter(isType=IS.SET_ENABLE_AUTO_SHUTTER, pval1=0, pval2=0)
        return r

    def enableAutoWhitebalance(self):
        ''' not supported yet by the driver '''
        r = self.SetAutoParameter(isType=IS.SET_ENABLE_AUTO_WHITEBALANCE, pval1=1, pval2=0)
        return r

    def disableAutoWhitebalance(self):
        ''' not supported yet by the driver '''
        r = self.SetAutoParameter(isType=IS.SET_ENABLE_AUTO_WHITEBALANCE, pval1=0, pval2=0)
        return r

    def getAutoGain(self):
        r = self.SetAutoParameter(isType=IS.GET_ENABLE_AUTO_GAIN, pval1=1, pval2=0)
        return bool(r['pval1'])


    def getAutoExposure(self):
        r = self.SetAutoParameter(isType=IS.GET_ENABLE_AUTO_SHUTTER, pval1=1, pval2=0)
        return bool(r['pval1'])

    def setAutoGain(self, enable):
        if enable:
            return self.enableAutoGain()
        else:
            return self.disableAutoGain()

    def setAutoExposure(self, enable):
        if enable:
            return self.enableAutoExposure()
        else:
            return self.disableAutoExposure()

    def toggleAutoParameters(self):
        r = self.enableAutoGain()
        r = self.enableAutoExposure()
        time.sleep(.2)
        r = self.disableAutoGain()
        r = self.disableAutoExposure()

    def SetFrameRate(self, fps=30):
        fps_c = ctypes.c_double(fps)
        new_fps_c = ctypes.c_double(0)
        r = CALL('SetFrameRate', self, fps_c, byref(new_fps_c))
        if r == 0:
            return new_fps_c.value

    def GetFrameRate(self):
        return self.SetFrameRate(fps=IS.GET_FRAMERATE)
        

    def enum_rates(self):
        rates = []
        for n in [1000.0/25, 1000.0/30, 1000.0/60,1000.0/100,1000.0/200]:
            rates.append((n,1000.0))
        return rates

    def set_rate(self, fps):
        try:
            self.current_rate_idx = self.rates.index(fps)
        except ValueError:
            logger.warning("Buggy Video Camera: Not all available rates are exposed.")
            self.current_rate_idx = 0
        logger.info("new frame rate " + str(fps[1]/fps[0]))
        return self.SetFrameRate(fps[1]/fps[0])

    def get_rate(self):
        return self.GetFrameRate()


    def set_rate_idx(self,rate_id):
        new_rate = self.rates[rate_id]
        r = self.set_rate(new_rate)
        return r

    def get_size(self):
        return self.GetAOI()

    def set_size(self, isType=IS.SET_IMAGE_AOI, x=0, y=0, width=260, height=216):
        self.SetAOI()
        time.sleep(.2)
        self.toNumpyArray()
        self.init_frame = self.data.copy()
        r = self.SetAOI(isType=IS.SET_IMAGE_AOI, x=x, y=y, width=width, height=height)
        self.toggleAutoParameters()
        return r
        
    def cleanup(self):
        self.StopLiveVideo()
        self.FreeImageMem()
        self.ExitCamera()


    def isOpened(self):
        return True

    def enableEvent(self, event=IS.SET_EVENT_FRAME):
        which = ctypes.c_int(event)
        r = CALL('EnableEvent', self, which)

    def waitForNewFrame(self):
        ''' wait until new frame is available '''
        which = ctypes.c_int(IS.SET_EVENT_FRAME)
        timeout = ctypes.c_int(1000)
        r = CALL('WaitEvent', self, which, timeout)
        return True

    # def read(self):
    #     """
    #     wrapper to make analogous to opencv call
    #     return most recent frame
    #     """
    #     self.waitForNewFrame()
    #     now = time.time()
    #     self.toNumpyArray()
    #     # frame = self.init_frame.copy()
    #     # frame[self.t_roi[0]:self.t_roi[2],self.t_roi[1]:self.t_roi[3]] = self.data[self.roi[0]:self.roi[2],self.roi[1]:self.roi[3]]
    #
    #     frame = cv2.cvtColor( self.data, cv2.COLOR_GRAY2RGB )
    #     return Frame(now, frame) #self.init_frame)

    def enum_sizes(self):
        return([(260,216)])


    # def set_exposure_time(self, exp):
    #     ''' set exposure time of camera '''
    #     new_exp = ctypes.c_double(0)
    #     exp = ctypes.c_double(exp)
    #     r = CALL('SetExposureTime', self, exp, byref(new_exp))
    #     if r is SUCCESS:
    #         return new_exp.value
    #     else:
    #         return -1

    def set_exposure_time(self, exp):
        ''' set exposure time of camera '''
        new_exp = ctypes.c_double(0)
        exp = ctypes.c_double(exp)
        r = CALL('SetExposureTime', self, exp, byref(new_exp))
        if r is SUCCESS:
            return new_exp.value
        else:
            return 0 

    def get_exposure_time(self):
        ''' gets exposure time of camera '''
        new_exp = ctypes.c_double(0)
        exp = ctypes.c_double(IS.GET_EXPOSURE_TIME)
        r = CALL('SetExposureTime', self, exp, byref(new_exp))
        if r is SUCCESS:
            return new_exp.value
        else:
            return 0 

    def set_contrast(self, cont):
        ''' set contrast from 0 - 200%'''
        cont = int(cont)
        cont = max(0,cont)
        cont = min(511, cont)
        cont = ctypes.c_int(cont)
        r = CALL('SetContrast', self, cont)
        if r is SUCCESS:
            return True
        else:
            return False

    def get_contrast(self):
        r = CALL('SetContrast', self, IS.GET_CONTRAST)
        return r

    def get_gain_boost(self, mode=IS.GET_GAINBOOST):
        r = CALL('SetGainBoost', self, mode)
        if r == IS.SET_GAINBOOST_ON:
            return True
        else:
            return False

    def set_gain_boost(self, enable=True):
        if enable:
            r = CALL('SetGainBoost', self, IS.SET_GAINBOOST_ON)
        else:
            r = CALL('SetGainBoost', self, IS.SET_GAINBOOST_OFF)
        if r == IS.SET_GAINBOOST_ON:
            return True
        else:
            return False

    def set_brightness(self, bright):
        '''
        Sets brightness from 0 - 200%
        '''
        bright = max(0,bright)
        bright = min(255, bright)
        bright = ctypes.c_int(bright)
        r = CALL('SetBrightness', self, bright)
        if r is SUCCESS:
            return True
        else:
            return False

    def get_brightness(self):
        r = CALL('SetBrightness', self, IS.GET_BRIGHTNESS)
        return r
        
        

    """
    
    Functions added by BM
    
    """
    
    def SetPixelClock(self, clock=30):
        """
        Sets the pixel clock frequency for this camera.

        :param clock: clock frequency (MHz)
        :type clock: integer
        """
        clock_c = ctypes.c_uint(clock)
        r = CALL('PixelClock', self, IS.PIXELCLOCK_CMD_SET, byref(clock_c), ctypes.sizeof(clock_c))
        return self.CheckForSuccessError(r)
    
    def GetPixelClock(self):
        """
        Gets the pixel clock frequency of this camera.

        :return: clock frequency (MHz)
        :rtype: integer
        """
        clock_c = ctypes.c_uint(0)
        r = CALL('PixelClock', self, IS.PIXELCLOCK_CMD_GET, byref(clock_c), ctypes.sizeof(clock_c))
        if r == 0:        
            return clock_c.value
    
    def SetBinning(self, mode=IS.BINNING_DISABLE):
        """
        Sets the binning mode, direction, and factor for this camera.

        :param mode: binning mode (IS.BINNING_DISABLE | IS.BINNING_2X_HORIZONTAL | ... | IS.BINNING_4X_VERTICAL )
        :type mode: integer
        """
        isType_c = ctypes.c_int(mode)
        r = CALL('SetBinning', self, isType_c)
        return self.CheckForSuccessError(r)
        
    def set_hardware_gain(self, nMaster, nRed, nGreen, nBlue):
        """
        Sets the gains for this camera.

        :param nMaster: master gain (0-100)
        :type nMaster: integer
        :param nRed: red gain (0-100)
        :type nRed: integer
        :param nGreen: green gain (0-100)
        :type nGreen: integer
        :param nBlue: blue gain (0-100)
        :type nBlue: integer
        """
        nMaster_c = ctypes.c_int(nMaster)
        nRed_c = ctypes.c_int(nRed)
        nGreen_c = ctypes.c_int(nGreen)
        nBlue_c = ctypes.c_int(nBlue)
        #ignore_me = ctypes.c_double(IS.IGNORE_PARAMETER)
        r = CALL('SetHardwareGain', self, nMaster_c, nRed_c, nGreen_c, nBlue_c)
        if r is SUCCESS:
            return True
        else:
            return False

    def get_hardware_gain(self):
        """
        Gets the master gain for this camera.

        :return: master gain (0-100)
        :rtype: integer
        """
        gMaster = CALL('SetHardwareGain', self, IS.GET_MASTER_GAIN)
        return gMaster
    
    def get_red_gain(self):
        """
        Gets the red gain for this camera.

        :return: red gain (0-100)
        :rtype: integer
        """
        gRed = CALL('SetHardwareGain', self, IS.GET_RED_GAIN)
        return gRed
    
    def get_green_gain(self):
        """
        Gets the green gain for this camera.

        :return: green gain (0-100)
        :rtype: integer
        """
        gGreen = CALL('SetHardwareGain', self, IS.GET_GREEN_GAIN)
        return gGreen
    
    def get_blue_gain(self):
        """
        Gets the blue gain for this camera.

        :return: blue gain (0-100)
        :rtype: integer
        """
        gBlue = CALL('SetHardwareGain', self, IS.GET_BLUE_GAIN)
        return gBlue

