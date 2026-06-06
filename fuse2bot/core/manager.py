import os

import adsk.fusion

from . import parser
from . import io


class Manager:
    ''' Manager class for setting params and generating URDF 
    '''    

    root = None 
    design = None
    _app = None

    UNIT_SCALE = {
        'mm': 0.001,
        'cm': 0.01,
        'm': 1.0,
    }

    INERTIA_ACCURACY = {
        'Low': adsk.fusion.CalculationAccuracy.LowCalculationAccuracy,
        'Medium': adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy,
        'High': adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy,
    }

    MESH_ACCURACY = {
        'Low': adsk.fusion.MeshRefinementSettings.MeshRefinementLow,
        'Medium': adsk.fusion.MeshRefinementSettings.MeshRefinementMedium,
        'High': adsk.fusion.MeshRefinementSettings.MeshRefinementHigh,
    }

    def __init__(self, save_dir, save_mesh, sub_mesh, mesh_resolution, inertia_precision,
                document_units, target_units, joint_order, target_platform) -> None:
        '''Initialization of Manager class 

        Parameters
        ----------
        save_dir : str
            path to directory for storing data
        save_mesh : bool
            if mesh data should be exported
        mesh_resolution : str
            quality of mesh conversion
        inertia_precision : str
            quality of inertia calculations
        document_units : str
            base units of current file
        target_units : str
            target files units
        joint_order : str
            if parent or child should be component 1
        target_platform : str
            which configuration to use for exporting urdf

        '''        
        self.save_mesh = save_mesh
        self.sub_mesh = sub_mesh
        self.scale = self._unit_scale(target_units) / self._unit_scale(document_units)
        self.inert_accuracy = self._lookup(self.INERTIA_ACCURACY, inertia_precision, 'inertia precision')
        self.mesh_accuracy = self._lookup(self.MESH_ACCURACY, mesh_resolution, 'mesh resolution')

        if joint_order == 'Parent':
            self.joint_order = ('p','c')
        elif joint_order == 'Child':
            self.joint_order = ('c','p')
        else:
            raise ValueError(f'Order method not supported')
        
        # set the target platform
        self.target_platform = target_platform

        # Set directory 
        self._set_dir(save_dir)

    @classmethod
    def _lookup(cls, values, key, label):
        try:
            return values[key]
        except KeyError as exc:
            raise ValueError(f'Unsupported {label}: {key}') from exc

    @classmethod
    def _unit_scale(cls, unit):
        return cls._lookup(cls.UNIT_SCALE, unit, 'unit')

    def _set_dir(self, save_dir):
        '''sets the class instance save directory

        Parameters
        ----------
        save_dir : str
            path to save
        '''        
        # set the names        
        robot_name = Manager.root.name.split()[0]
        package_name = robot_name + '_description'

        self.save_dir = os.path.join(save_dir, package_name)
        os.makedirs(self.save_dir, exist_ok=True)

    def preview(self):
        ''' Get all joints in the scene for previewing joints

        Returns
        -------
        dict
            mapping of joint names with parent-> child relationship
        '''        
        
        config = parser.Configurator(Manager.root)
        config.inertia_accuracy = self.inert_accuracy
        config.joint_order = self.joint_order
        config.scale = self.scale
        config.target_platform = self.target_platform
        ## Return array of tuples (parent, child)
        config.get_scene_configuration()
        return config.get_joint_preview()


    def run(self):
        ''' process the scene, including writing to directory and
        exporting mesh, if applicable
        '''        
        
        config = parser.Configurator(Manager.root)
        config.inertia_accuracy = self.inert_accuracy
        config.scale = self.scale
        config.joint_order = self.joint_order
        config.sub_mesh = self.sub_mesh
        config.target_platform = self.target_platform
        config.get_scene_configuration()
        config.parse()

        # --------------------
        # Generate URDF
        writer = io.Writer()
        writer.write_urdf(self.save_dir, config, self.target_platform)

        if self.target_platform == 'pyBullet':
            io.write_hello_pybullet(config.name, self.save_dir)
        
        # Custom STL Export
        if self.save_mesh:
            io.visible_to_stl(
                Manager.design,
                self.save_dir,
                Manager.root,
                self.mesh_accuracy,
                config.body_dict,
                self.sub_mesh,
                config.body_mapper,
                Manager._app,
                self.target_platform,
            )

