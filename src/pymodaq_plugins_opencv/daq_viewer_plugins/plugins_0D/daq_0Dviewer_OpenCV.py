import numpy as np
import cv2
import logging
from pymodaq.utils.daq_utils import ThreadCommand
from pymodaq.utils.data import DataFromPlugins, DataToExport
from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters, main
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_opencv.hardware.daq_opencv import OpenCVProp as Focus

logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING)


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
        ## TODO for your custom plugin: elements to be added here as dicts in order to control your custom stage
        ]},]

    def ini_attributes(self):
        #  TODO declare the type of the wrapper (and assign it to self.controller) you're going to use for easy
        #  autocompletion
        self.controller: Focus = None #TODO: Why cv2.VideoCapture? Shouldn't it be Focus?
        self.x_axis = None
        self.y_axis = None

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        ## TODO for your custom plugin
        if param.name() == "open_settings":
           if param.value():
               self.controller.set(Focus['CV_CAP_' + param.name()].value, param.value())
#        elif ...
        ##

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

        raise NotImplemented  # TODO when writing your own plugin remove this line and modify the one below
        self.ini_detector_init(slave_controller=controller)

        if self.is_master:
            self.controller = PythonWrapperOfYourInstrument()  #instantiate you driver with whatever arguments are needed
            self.controller.open_communication() # call eventual methods

        # TODO for your custom plugin (optional) initialize viewers panel with the future type of data
        self.dte_signal_temp.emit(DataToExport(name='myplugin',
                                               data=[DataFromPlugins(name='Mock1',
                                                                    data=[np.array([0]), np.array([0])],
                                                                    dim='Data0D',
                                                                    labels=['Mock1', 'label2'])]))

        info = "Whatever info you want to log"
        initialized = self.controller.a_method_or_atttribute_to_check_if_init()  # TODO
        return info, initialized

    def close(self):
        """Terminate the communication protocol"""
        ## TODO for your custom plugin
        raise NotImplemented  # when writing your own plugin remove this line
        #  self.controller.your_method_to_terminate_the_communication()  # when writing your own plugin replace this line

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
        focus, frame = self.controller.read()

        if focus: 
            if self.settings['color'] == 'grey':
                camera = cv2.GaussianBlur(frame, (3, 3), 0)
                camera = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                camera = self.laplacian(camera)
                camera[0] = camera[0].astype(np.float32)
            else: 
                if len(frame.shape) == 2: 
                    camera = cv2.cvtColor(frame[:,:,ind] for ind in range(frame.shape[2]))
                else:
                    camera = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)]
        else: 
            camera = [np.zero((len(self.y_axis), len(self.x_axis)))]
            logger.warning('No frame grabbed')
    
        self.dte_signal.emit(DataToExport(name='myplugin',
                                          data=[DataFromPlugins(name='OpenCV', data=camera,
                                                                dim='Data0D', labels=['focus', 'frame'])]))
        #########################################################

        # # asynchrone version (non-blocking function with callback)
        # raise NotImplemented  # when writing your own plugin remove this line
        # self.controller.your_method_to_start_a_grab_snap(self.callback)  # when writing your own plugin replace this line
        # #########################################################
    def laplacian(self, frame): 
        laplacian = cv2.Laplacian(frame, cv2.CV_64F)
        laplacian = cv2.convertScaleAbs(laplacian)
        return laplacian
   
    


    def callback(self):
        """optional asynchrone method called when the detector has finished its acquisition of data"""
        data_tot = self.controller.your_method_to_get_data_from_buffer()
        self.dte_signal.emit(DataToExport(name='myplugin',
                                          data=[DataFromPlugins(name='Mock1', data=data_tot,
                                                                dim='Data0D', labels=['dat0', 'data1'])]))

    def stop(self):
        """Stop the current grab hardware wise if necessary"""
        ## TODO for your custom plugin
        raise NotImplemented  # when writing your own plugin remove this line
        self.controller.your_method_to_stop_acquisition()  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))
        ##############################
        return ''


if __name__ == '__main__':
    main(__file__)