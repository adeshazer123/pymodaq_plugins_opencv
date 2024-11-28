import numpy as np
import cv2
import logging
from pymodaq.utils.daq_utils import ThreadCommand
from pymodaq.utils.data import DataFromPlugins, DataToExport
from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters, main
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_opencv.hardware.daq_opencv import OpenCVProp as Focus  # DK - correct the file name

logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING) # DK - Delete. We should not change the default setting.


# TODO:
# (1) change the name of the following class to DAQ_0DViewer_TheNameOfYourChoice
# (2) change the name of this file to daq_0Dviewer_TheNameOfYourChoice ("TheNameOfYourChoice" should be the SAME
#     for the class name and the file name.)
# (3) this file should then be put into the right folder, namely IN THE FOLDER OF THE PLUGIN YOU ARE DEVELOPING:
#     pymodaq_plugins_my_plugin/daq_viewer_plugins/plugins_0D
class DAQ_0DViewer_OpenCV(DAQ_Viewer_base):
    """ Instrument plugin class for a OD viewer.
    
    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Viewer module through inheritance via
    DAQ_Viewer_base. It makes a bridge between the DAQ_Viewer module and the Python wrapper of a particular instrument.

    TODO Complete the docstring of your plugin with:
        * The set of instruments that should be compatible with this instrument plugin.
        * With which instrument it has actually been tested.
        * The version of PyMoDAQ during the test.
        * The version of the operating system.
        * Installation instructions: what manufacturer’s drivers should be installed to make it run?

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library.
         
    # TODO add your particular attributes here if any

    """
    params = comon_parameters+[
        {'title': 'Camera index', 'name': 'camera_index', 'type': 'int', 'value': 0, 'default': 0, 'min': 0},
        {'title': 'Color', 'name': 'color', 'type': 'list', 'value': 'grey', 'limits':['grey', 'RGB']}, 
        {'title': 'Open Settings', 'name': 'open_settings', 'type': 'bool', 'value': False},
        {'title': 'Camera Settings', 'name': 'cam_settings', 'type': 'group', 'children': [
        ]},]

    def ini_attributes(self):
        self.controller: cv2.VideoCapture = None 
        self.x_axis = None
        self.y_axis = None

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == "open_settings":
           if param.value():
               self.controller.set(Focus['CV_CAP_PROP_SETTINGS'].value, 1)
        elif param.name() == 'color':
             pass 
        else:
            self.controller.set(Focus['CV_CAP_' + param.name()].value, param.value())
            val = self.controller.get(Focus['CV_CAP_' + param.name()].value)
            param.setValue(val) 
    
    def get_active_properties(self):
        props = Focus.names()
        self.additional_params = []
        for prop in props:
            try:
                ret = int(self.controller.get(Focus[prop].value))
                if ret != -1:
                    try:
                        ret_set = self.controller.set(Focus[prop].value, ret)
                    except:
                        ret_set = False
                    self.additional_params.append(
                        {'title': prop[7:], 'name': prop[7:], 'type': 'float', 'value': ret, 'readonly': not ret_set})
            except:
                pass
        self.settings.child('cam_settings').addChildren(self.additional_params)
        pass

    def ini_detector(self, controller=None):
        """Detector communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator/detector by controller
            (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """

        self.ini_detector_init(slave_controller=controller)

        if self.is_master:
            self.controller = cv2.VideoCapture(self.settings['camera_index'], cv2.CAP_DSHOW)
            self.x_axis = self.get_xaxis()
            self.y_axis = self.get_yaxis()
            self.get_active_properties()
            

        self.dte_signal_temp.emit(DataToExport(name='myplugin',
                                               data=[DataFromPlugins(name='FocusFinder', # rename Mock1
                                                                    data=[np.array([0]), np.array([0])],
                                                                    dim='Data0D',
                                                                    labels=[self.x_axis, self.y_axis])]))

        info = "OpenCV 0D viewer initialized"
        initialized = True
        return info, initialized
    
    def get_xaxis(self):
        xaxis = int(self.controller.get(cv2.CAP_PROP_FRAME_WIDTH))
        return xaxis
    
    def get_yaxis(self):
        yaxis = int(self.controller.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return yaxis
    
    def close(self):
        """Terminate the communication protocol"""
        self.controller.release() 

    def grab_data(self, Naverage=1, **kwargs):
        """Start a grab from the detector

        Parameters
        ----------
        Naverage: int
            Number of hardware averaging (if hardware averaging is possible, self.hardware_averaging should be set to
            True in class preamble and you should code this implementation)
        kwargs: dict
            others optionals arguments
        """
        if not self.controller.isOpened():
            self.controller.open(self.settings['camera_index'])
            
        ret, frame = self.controller.read()

        if ret: 
            if self.settings['color'] == 'grey':
                camera = cv2.GaussianBlur(frame, (3, 3), 0)
                camera = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                camera = self.laplacian(camera)
                camera[0] = camera[0].astype(np.float32)
            else: 
                if len(frame.shape) == 3: 
                    camera = cv2.cvtColor(frame[:,:,ind] for ind in range(frame.shape[2]))
                else:
                    camera = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)]
        else: 
            camera = [np.zero((len(self.y_axis), len(self.x_axis)))]
            logger.warning('No frame grabbed')
    
        self.dte_signal.emit(DataToExport(name='FocusFinder', 
                                          data=[DataFromPlugins(name='OpenCV', data=camera,
                                                                dim='Data0D', labels=['focus'])]))
        
    
    def calculate_score(self, frame): 
        """Calculate the focus score of the frame"""
        laplacian = cv2.Laplacian(frame, cv2.CV_64F)
        focus_score: float = laplacian.var()
        return focus_score
   
    
    def callback(self): # DK - do we need this? Either synchronous and synchronous works because the exposure time of this camera is short.
        """optional asynchrone method called when the detector has finished its acquisition of data"""
        data_tot = self.controller.your_method_to_get_data_from_buffer() # DK - edit this placeholder
        self.dte_signal.emit(DataToExport(name='myplugin',
                                          data=[DataFromPlugins(name='Mock1', data=data_tot,
                                                                dim='Data0D', labels=['dat0', 'data1'])]))

    def stop(self):
        """Stop the current grab hardware wise if necessary"""
        pass


if __name__ == '__main__':
    main(__file__)