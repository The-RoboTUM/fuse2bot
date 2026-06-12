import csv
import os

import adsk.core
import adsk.fusion

from . import parser
from . import transforms
from . import utils
from collections import Counter

def visible_to_stl(design, save_dir, root, accuracy, body_dict, sub_mesh, body_mapper, _app, target_platform='None'):
    """
    export top-level components as a single stl file into "save_dir/"
    
    Parameters
    ----------
    design: adsk.fusion.Design
        fusion design document
    save_dir: str
        directory path to save
    root: adsk.fusion.Component
        root component of the design
    accuracy: int
        accuracy value to use for stl export
    component_map: list
        list of all bodies to use for stl export
    """
          
    # get the script location
    save_dir = os.path.join(save_dir, 'meshes')
    os.makedirs(save_dir, exist_ok=True)

    # Export top-level occurrences
    occ = root.occurrences.asList
    # hack for correct stl placement by turning off all visibility first
    visible_components = []
    for oc in occ:
        if oc.isLightBulbOn:
            visible_components.append(oc)

    # Make sure no repeated body names
    body_count = Counter()

    new_doc = None
    try:
        # Use a throwaway design so temporary export bodies never touch the source robot document.
        new_doc = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType, True)
        new_design: adsk.fusion.Design = new_doc.products.itemByProductType('DesignProductType')
        new_root = new_design.rootComponent
        exporter = new_design.exportManager
        mesh_transform = isaac_mesh_transform() if target_platform == 'IsaacSim' else None

        for oc in visible_components:
            occ_name = utils.format_urdf_name(oc.name)

            component_exporter(
                exporter,
                new_root,
                body_mapper[oc.entityToken],
                os.path.join(save_dir, f'{occ_name}'),
                mesh_transform,
            )

            if sub_mesh:
                # get the bodies associated with this top-level component (which will contain sub-components)
                bodies = body_mapper[oc.entityToken]

                for body in bodies:
                    if body.isLightBulbOn:

                        # Since there are alot of similar names, we need to store the parent component as well in the filename
                        body_name = utils.format_urdf_name(body.name)
                        body_name_cnt = f'{body_name}_{body_count[body_name]}'
                        body_count[body_name] += 1

                        save_name = os.path.join(save_dir, f'{occ_name}_{body_name_cnt}')

                        body_exporter(exporter, new_root, body, save_name, mesh_transform)
    finally:
        if new_doc is not None:
            new_doc.close(False)


def isaac_mesh_transform():
    transform = adsk.core.Matrix3D.create()
    axis = adsk.core.Vector3D.create(1, 0, 0)
    origin = adsk.core.Point3D.create(0, 0, 0)
    transform.setToRotation(transforms.ISAAC_ROTATION_RPY[0], axis, origin)
    return transform


def component_exporter(export_mgr, new_root, body_lst, filename, mesh_transform=None):
    ''' Copy a component to a new document, save, then delete. 

    Modified from solution proposed by BrianEkins https://EkinsSolutions.com

    Parameters
    ----------
    exportMgr : _type_
        _description_
    newRoot : _type_
        _description_
    body_lst : _type_
        _description_
    filename : _type_
        _description_
    '''

    tBrep = adsk.fusion.TemporaryBRepManager.get()

    bf = new_root.features.baseFeatures.add()
    bf.startEdit()

    for body in body_lst:
        if not body.isLightBulbOn:
            continue
        tmp_body = tBrep.copy(body)
        if mesh_transform is not None:
            tBrep.transform(tmp_body, mesh_transform)

        new_root.bRepBodies.add(tmp_body, bf)

    bf.finishEdit()
    stl_options = export_mgr.createSTLExportOptions(new_root, f'{filename}.stl')
    export_mgr.execute(stl_options)

    bf.deleteMe()

def body_exporter(export_mgr, new_root, body, filename, mesh_transform=None):
    tBrep = adsk.fusion.TemporaryBRepManager.get()

    tmp_body = tBrep.copy(body)
    if mesh_transform is not None:
        tBrep.transform(tmp_body, mesh_transform)

    bf = new_root.features.baseFeatures.add()
    bf.startEdit()
    new_root.bRepBodies.add(tmp_body, bf)
    bf.finishEdit()

    new_body = new_root.bRepBodies[0]

    stl_options = export_mgr.createSTLExportOptions(new_body, filename)
    stl_options.sendToPrintUtility = False
    stl_options.isBinaryFormat = True
    # stl_options.meshRefinement = accuracy
    export_mgr.execute(stl_options)

    bf.deleteMe()

class Writer:

    def __init__(self) -> None:
        pass

    def write_link(self, config, file_name):
        ''' Write links information into urdf file_name
        
        Parameters
        ----------
        config : Configurator
            root nodes instance of configurator class
        file_name: str
            urdf full path

        '''

        with open(file_name, mode='a', encoding="utf-8") as f:
            for _, link in config.links.items():
                f.write(f'{link.link_xml}\n')

    def write_joint(self, file_name, config: parser.Configurator):
        ''' Write joints and transmission information into urdf file_name
            
        Parameters
        ----------
        file_name: str
            urdf full path
        config : Configurator
            root nodes instance of configurator class

        '''
        
        with open(file_name, mode='a', encoding="utf-8") as f:
            for _, joint in config.joints.items():
                f.write(f'{joint.joint_xml}\n')

    def write_materials(self, f, config: parser.Configurator):
        '''Write robot-level visual materials.'''

        materials = {
            'silver': (0.7, 0.7, 0.7, 1.0),
        }
        for material_info in config.visual_materials.values():
            name = material_info.get('name')
            rgba = material_info.get('rgba')
            if name and rgba:
                materials[name] = rgba

        for name, rgba in materials.items():
            f.write(f'<material name="{name}">\n')
            f.write(f'  <color rgba="{" ".join([str(_) for _ in rgba])}"/>\n')
            f.write('</material>\n\n')

    def write_material_report(self, file_name, config: parser.Configurator):
        '''Write body material and mass diagnostics next to the URDF.'''

        fieldnames = [
            'link',
            'occurrence',
            'body',
            'visible',
            'included_in_exported_mass',
            'physical_material',
            'appearance',
            'rgba',
            'body_mass_kg',
            'density',
            'volume',
        ]

        with open(file_name, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in config.material_report_rows:
                report_row = dict(row)
                rgba = report_row.get('rgba')
                if rgba:
                    report_row['rgba'] = ' '.join([str(_) for _ in rgba])
                writer.writerow(report_row)


    def write_urdf(self, save_dir, config: parser.Configurator, target_platform='None'):
        ''' Write each component of the xml structure to file

        Parameters
        ----------
        save_dir : str
            path to save file
        config : Configurator
            root nodes instance of configurator class
        '''        

        save_dir = os.path.join(save_dir, 'urdf')
        os.makedirs(save_dir, exist_ok=True)
        robot_name = utils.format_urdf_name(config.name)
        file_name = os.path.join(save_dir, f'{robot_name}.urdf')  # the name of urdf file
        material_report_name = os.path.join(save_dir, f'{robot_name}_materials.csv')

        with open(file_name, mode='w', encoding="utf-8") as f:
            f.write('<?xml version="1.0" ?>\n')
            f.write(f'<robot name="{robot_name}">\n\n')
            self.write_materials(f, config)

            if target_platform == 'IsaacSim':
                f.write('<!-- Coordinates baked for Isaac Sim: +X forward, +Z up. -->\n\n')

        self.write_link(config, file_name)
        self.write_joint(file_name, config)
        self.write_material_report(material_report_name, config)

        with open(file_name, mode='a', encoding="utf-8") as f:
            f.write('</robot>\n')

def write_hello_pybullet(robot_name, save_dir):
    ''' Writes a sample script which loads the URDF in pybullet

    Modified from https://github.com/yanshil/Fusion2PyBullet

    Parameters
    ----------
    robot_name : str
        name to use for directory
    save_dir : str
        path to store file
    '''    

    robot_urdf = f'{robot_name}.urdf'  # basename of robot.urdf
    file_name = os.path.join(save_dir, 'hello_bullet.py')
    hello_pybullet = """
import pybullet as p
import os
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
dir = os.path.abspath(os.path.dirname(__file__))
robot_urdf = "TEMPLATE.urdf"
dir = os.path.join(dir,'urdf')
robot_urdf=os.path.join(dir,robot_urdf)
robotId = p.loadURDF(robot_urdf,cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()
"""
    hello_pybullet = hello_pybullet.replace('TEMPLATE.urdf', robot_urdf)
    with open(file_name, mode='w', encoding="utf-8") as f:
        f.write(hello_pybullet)
        f.write('\n')
