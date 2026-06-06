'''
module to parse fusion file 
'''

import adsk.fusion

from collections import Counter, defaultdict

from . import transforms
from . import parts
from . import utils

class Hierarchy:
    ''' hierarchy of the design space '''

    def __init__(self, component) -> None:
        ''' Initialize Hierarchy class to parse document and define component relationships.
        Uses a recursive traversal (based off of fusion example) and provides helper functions
        to get specific children and parents for nodes. 
        Parameters
        ----------
        component : [type]
            fusions root component to use for traversal
        '''        

        self.children = []
        self.component = component
        self.name = component.name
        self._parent = None

    def _add_child(self, c):
        self.children.append(c)
        c.parent = self

    def get_children(self):
        return self.children        

    def get_all_children(self):
        ''' get all children and sub children of this instance '''

        child_map = {}
        parent_stack = set()
        parent_stack.update(self.get_children())
        while len(parent_stack) != 0:
            # Pop an element form the stack (order shouldn't matter)
            tmp = parent_stack.pop()
            # Add this child to the map
            # use the entity token, more accurate than the name of the component (since there are multiple)
            child_map[tmp.component.entityToken] = tmp 
            # Check if this child has children
            if len(tmp.get_children()) > 0:
                # add them to the parent_stack
                parent_stack.update(tmp.get_children())

        return child_map

    def get_flat_body(self):
        ''' get a flat list of all components and child components '''

        body_list = []
        parent_stack = set()

        child_set = list(self.get_all_children().values())

        if len(child_set) == 0:
            body_list.append([self.component.bRepBodies.item(x) for x in range(0, self.component.bRepBodies.count) ])

        child_list = [x.children for x in child_set if len(x.children) > 0]
        childs = []
        for c in child_list:
            for _c in c:
                childs.append(_c)

        parent_stack.update(childs)
        closed_set = set()

        while len(parent_stack) != 0:
            # Pop an element form the stack (order shouldn't matter)
            tmp = parent_stack.pop()
            closed_set.add(tmp)
            # Get any bodies directly associated with this component
            if tmp.component.bRepBodies.count > 0:
                body_list.append([tmp.component.bRepBodies.item(x) for x in range(0, tmp.component.bRepBodies.count) ])

            # Check if this child has children
            if len(tmp.children) > 0:
                # add them to the parent_stack
                child_set = list(self.get_all_children().values())

                child_list = [x.children for x in child_set if len(x.children) > 0]
                childs = []
                for c in child_list:
                    for _c in c:
                        if _c not in closed_set:
                            childs.append(_c)

                parent_stack.update(childs)

        flat_bodies = []
        for body in body_list:
            flat_bodies.extend(body)

        return flat_bodies

    def get_all_parents(self):
        ''' get all the parents of this instance '''

        child_stack = set()
        child_stack.add(self)
        parent_map = []
        while len(child_stack) != 0:
            tmp = child_stack.pop()
            if tmp.parent is None:
                return parent_map
            parent_map.append(tmp.parent.component.entityToken)    
            child_stack.add(tmp.parent)

        return parent_map
            
    @property
    def parent(self):
        if self._parent is None:
            return None
        return self._parent

    @parent.setter
    def parent(self,v):
        self._parent = v

    @staticmethod
    def traverse(occurrences, parent=None):
        '''Recursively create class instances and define a parent->child structure
        Based on the fusion 360 API docs
        
        Parameters
        ----------
        occurrences : [type]
            [description]
        parent : [type], optional
            [description], by default None
        Returns
        -------
        Hierarchy
            Instance of the class
        '''        
        
        cur = parent
        for i in range(0, occurrences.count):
            occ = occurrences.item(i)
            cur = Hierarchy(occ)

            if parent is not None:
                parent._add_child(cur)

            if occ.childOccurrences:
                Hierarchy.traverse(occ.childOccurrences, parent=cur)
        return cur

class Configurator:

    joint_type_list = [ 'fixed', 'revolute', 'prismatic', 'Cylinderical',
                        'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    def __init__(self, root) -> None:
        ''' Initializes Configurator class to handle building hierarchy and parsing
        Parameters
        ----------
        root : [type]
            root component of design document
        '''        
        # Export top-level occurrences
        self.root = root
        self.occ = root.occurrences.asList
        self.inertial_dict = {}
        self.inertia_accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy

        self.links_xyz_dict = {} # needed ?

        self.sub_mesh = False
        self.joints_dict = {}
        self.body_dict = {}
        self.links = {} # Link class
        self.virtual_links = [] # Store all the links identified having more than one parent
        self.corrected_positions = {} # store their corrected positions
        self.joints = {} # Joint class for writing to file
        self.joint_order = ('p','c') # Order of joints defined by components
        self.scale = 100.0 # Units to convert to meters (or whatever simulator takes)
        self.inertia_scale = 10000.0 # units to convert mass
        self.target_platform = 'None'
        self.base_links= set()
        # self.component_map = set()

        self.root_node = None

    @property
    def use_isaac_coordinates(self):
        return self.target_platform == 'IsaacSim'

    def _transform_xyz(self, vector):
        if self.use_isaac_coordinates:
            return transforms.fusion_y_up_to_isaac_z_up_xyz(vector)
        return vector

    def _transform_inertia(self, inertia):
        if self.use_isaac_coordinates:
            return transforms.fusion_y_up_to_isaac_z_up_inertia(inertia)
        return inertia

    def get_scene_configuration(self):
        '''Build the graph of how the scene components are related
        '''        
        
        self.root_node = Hierarchy(self.root)
        occ_list = self.root.occurrences.asList

        Hierarchy.traverse(occ_list, self.root_node)
        self.component_map = self.root_node.get_all_children()

        self.get_sub_bodies()

        return self.component_map



    def get_sub_bodies(self):
        ''' temp fix for ensuring that a top-level component is associated with bodies'''

        # write the immediate children of root node
        self.body_mapper = defaultdict(list)

        # for k,v in self.component_map.items():
        for v in self.root_node.children:
            
            children = set()
            children.update(v.children)

            top_level_body = [v.component.bRepBodies.item(x) for x in range(0, v.component.bRepBodies.count) ]
            top_level_body = [x for x in top_level_body if x.isLightBulbOn]
            
            # add to the body mapper
            self.body_mapper[v.component.entityToken].extend(top_level_body)

            while children:
                cur = children.pop()
                children.update(cur.children)
                sub_level_body = [cur.component.bRepBodies.item(x) for x in range(0, cur.component.bRepBodies.count) ]
                sub_level_body = [x for x in sub_level_body if x.isLightBulbOn ]
                
                # add to this body mapper again 
                self.body_mapper[v.component.entityToken].extend(sub_level_body)
                
    def get_joint_preview(self):
        ''' Get the scenes joint relationships without calculating links 
        Returns
        -------
        dict
            joint relationships
        '''

        self._joints()
        return self.joints_dict

    def parse(self):
        ''' parse the scene by building up inertia and joints'''

        self._inertia()
        self._joints()
        self._base()
        self._build_links()
        self._build_joints()

    @property
    def name(self):
        ''' Name of the root component '''
        return self.root.name.split()[0]

    def _base(self):
        ''' Get the base link(s) '''
        
        for oc in self.occ:
            if oc.isGrounded:
                name = oc.name
                self.base_links.add(name)

    def _inertia(self):
        '''
        Define inertia values
        
        Notes
        -----
        Original Authors: @syuntoku, @yanshil
        Modified by @cadop
        '''
        
        for oc in self.occ:       
            occs_dict = {}
            prop = oc.getPhysicalProperties(self.inertia_accuracy)
            
            occs_dict['name'] = oc.name

            mass = prop.mass  # kg

            # Iterate through bodies, only add mass of bodies that are visible (lightbulb)
            # body_cnt = oc.bRepBodies.count
            # mapped_comp =self.component_map[oc.entityToken]
            body_lst = self.component_map[oc.entityToken].get_flat_body()

            if len(body_lst) > 0:
                for body in body_lst:
                    # Check if this body is hidden
                    #  
                    # body = oc.bRepBodies.item(i)
                    if not body.isLightBulbOn:
                        mass -= body.physicalProperties.mass


            occs_dict['mass'] = mass
            center_of_mass = [_/self.scale for _ in prop.centerOfMass.asArray()] ## cm to m
            occs_dict['center_of_mass'] = self._transform_xyz(center_of_mass)


            # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
            (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
            moment_inertia_world = [_ / self.inertia_scale for _ in [xx, yy, zz, xy, yz, xz] ] ## kg / cm^2 -> kg/m^2

            inertia = transforms.origin2center_of_mass(moment_inertia_world, center_of_mass, mass)
            occs_dict['inertia'] = self._transform_inertia(inertia)

            self.inertial_dict[oc.name] = occs_dict

    def _is_joint_valid(self, joint):
        '''_summary_
        Parameters
        ----------
        joint : _type_
            _description_
        '''

        try: 
            joint.geometryOrOriginOne.origin.asArray()
            joint.geometryOrOriginTwo.origin.asArray()
            return True 
        
        except Exception:
            return False


    def _joints(self):
        ''' Iterates over joints list and defines properties for each joint
        (along with its relationship)
        
        '''        

        for joint in self.root.joints:
            
            joint_dict = {}
            joint_type = Configurator.joint_type_list[joint.jointMotion.jointType]
            joint_dict['type'] = joint_type

            # switch by the type of the joint
            joint_dict['axis'] = [0, 0, 0]
            joint_dict['upper_limit'] = 0.0
            joint_dict['lower_limit'] = 0.0

            occ_one = joint.occurrenceOne
            occ_two = joint.occurrenceTwo
            
            # Check that both bodies are valid (e.g. there is no missing reference)
            if not self._is_joint_valid(joint):
                continue

            geom_one_origin = joint.geometryOrOriginOne.origin.asArray()
            # Check if this is already top level
            # Check if the parent_list only contains one entity
            parent_list = self.component_map[occ_one.entityToken].get_all_parents()
            if len(parent_list) != 1:
                # the expectation is there is at least two items, the last is the full body
                # the second to last should be the next top-most component
                # reset occurrence one
                occ_one = self.component_map[parent_list[-2]].component

            parent_list = self.component_map[occ_two.entityToken].get_all_parents()
            if len(parent_list) != 1:
                # the expectation is there is at least two items, the last is the full body
                # the second to last should be the next top-most component
                # reset occurrence two
                occ_two = self.component_map[parent_list[-2]].component

            joint_type = joint.jointMotion.objectType # string 
            
            # Only Revolute joints have rotation axis 
            if 'RigidJointMotion' not in joint_type:
                if "RevoluteJointMotion" in joint_type:
                    joint_vector = self._transform_xyz(joint.jointMotion.rotationAxisVector.asArray())
                    joint_limit_max = joint.jointMotion.rotationLimits.maximumValue
                    joint_limit_min = joint.jointMotion.rotationLimits.minimumValue
                    
                    if abs(joint_limit_max - joint_limit_min) == 0:
                        joint_limit_min = -3.14159
                        joint_limit_max = 3.14159
                elif "SliderJointMotion" in joint_type:
                    # with open(r"C:\Programmieren\RoboTUM\fuse2bot\out\log" + f"{int(time.time())}.txt", "a") as log:
                    #     log.write(f"Slider has vars: {vars(joint.jointMotion)}\n")
                    #     log.write(f"Slider has dir: {dir(joint.jointMotion)}\n")
                    joint_vector = self._transform_xyz(joint.jointMotion.slideDirectionVector.asArray())
                    joint_limit_max = joint.jointMotion.slideLimits.maximumValue/self.scale
                    joint_limit_min = joint.jointMotion.slideLimits.minimumValue/self.scale
                    if joint_limit_max - joint_limit_min <= 0:
                        raise ValueError("SliderJoint with zero or negative travel detected!")
                else:
                    raise ValueError(f'Joint type {joint_type} not supported')

                # joint_rot_val = joint.jointMotion.rotationValue
                # joint_angle = joint.angle.value 

                joint_dict['axis'] = joint_vector
                joint_dict['upper_limit'] = joint_limit_max
                joint_dict['lower_limit'] = joint_limit_min

            # Reverses which is parent and child
            if self.joint_order == ('p','c'):
                joint_dict['parent'] = occ_one.name
                joint_dict['child'] = occ_two.name
            elif self.joint_order == ('c','p'):
                joint_dict['child'] = occ_one.name
                joint_dict['parent'] = occ_two.name
            else:
                raise ValueError(f'Order {self.joint_order} not supported')

            joint_dict['xyz'] = self._transform_xyz([x/self.scale for x in geom_one_origin])

            self.joints_dict[joint.name] = joint_dict

    def _build_links(self):
        ''' create links '''

        mesh_folder = 'meshes/'    

        #creates list of bodies that are visible

        self.body_dict = defaultdict(list) # key : occurrence name -> value : list of bodies under that occurrence
        body_dict_urdf = defaultdict(list) # list to send to parts.py
        oc_name = ''
        # Make sure no repeated body names
        body_count = Counter()
        
        for oc in self.occ:
            oc_name = utils.format_urdf_name(oc.name)
            # self.body_dict[oc_name] = []
            # body_lst = self.component_map[oc.entityToken].get_flat_body() #gets list of all bodies in the occurrence

            body_lst = self.body_mapper[oc.entityToken]
            
            if len(body_lst) > 0:
                for body in body_lst:
                    # Check if this body is hidden
                    if body.isLightBulbOn:
                        self.body_dict[oc_name].append(body)

                        body_name = utils.format_urdf_name(body.name)
                        body_name_cnt = f'{body_name}_{body_count[body_name]}'
                        body_count[body_name] += 1

                        unique_bodyname = f'{oc_name}_{body_name_cnt}'
                        body_dict_urdf[oc_name].append(unique_bodyname)
                    
        # Make the actual urdf names accessible
        self.body_dict_urdf = body_dict_urdf

        base_link = next(iter(self.base_links))
        base_urdf_name = utils.format_urdf_name(base_link)
        center_of_mass = self.inertial_dict[base_link]['center_of_mass']
        link = parts.Link(name=base_urdf_name, 
                        xyz=[0,0,0], 
                        center_of_mass=center_of_mass, 
                        sub_folder=mesh_folder,
                        mass=self.inertial_dict[base_link]['mass'],
                        inertia_tensor=self.inertial_dict[base_link]['inertia'],
                        body_dict = body_dict_urdf,
                        sub_mesh = self.sub_mesh)

        self.links_xyz_dict[base_urdf_name] = link.xyz
        self.links[base_urdf_name] = link

        def split_inertial(raw_name, ratio=0.5):
            mass = self.inertial_dict[raw_name]['mass']
            inertia = self.inertial_dict[raw_name]['inertia']

            real_mass = mass * (1.0 - ratio)
            virtual_mass = mass * ratio

            real_inertia = [x * (1.0 - ratio) for x in inertia]
            virtual_inertia = [x * ratio for x in inertia]

            return real_mass, virtual_mass, real_inertia, virtual_inertia

        for k, joint in self.joints_dict.items():
            raw_name = joint['child']
            name = utils.format_urdf_name(raw_name)

            if self.links.get(name) is not None:
                name_virtual = name + '_virtual'
                self.virtual_links.append(name_virtual)
                joint['child'] = name_virtual

                center_of_mass = [
                    i - j for i, j in zip(
                        self.inertial_dict[raw_name]['center_of_mass'],
                        joint['xyz']
                    )
                ]

                real_mass, virtual_mass, real_inertia, virtual_inertia = split_inertial(
                    raw_name,
                    ratio=0.5
                )

                # Share inertial properties with existing real link
                self.links[name].mass = real_mass
                self.links[name].inertia_tensor = real_inertia

                link = parts.Link(
                    name=name_virtual,
                    xyz=(joint['xyz'][0], joint['xyz'][1], joint['xyz'][2]),
                    center_of_mass=center_of_mass,
                    sub_folder=mesh_folder,
                    mass=virtual_mass,
                    inertia_tensor=virtual_inertia,
                    body_dict=body_dict_urdf,
                    sub_mesh=self.sub_mesh
                )

            else:
                joint['child'] = name
                center_of_mass = [ i-j for i, j in zip(self.inertial_dict[raw_name]['center_of_mass'], joint['xyz'])]
                link = parts.Link(name=name, 
                                xyz=(joint['xyz'][0], joint['xyz'][1], joint['xyz'][2]),
                                center_of_mass=center_of_mass,
                                sub_folder=mesh_folder, 
                                mass=self.inertial_dict[raw_name]['mass'],
                                inertia_tensor=self.inertial_dict[raw_name]['inertia'],
                                body_dict = body_dict_urdf,
                                sub_mesh = self.sub_mesh)

            self.links_xyz_dict[link.name] = (link.xyz[0], link.xyz[1], link.xyz[2])   
            self.links[link.name] = link



    def _build_joints(self):
        ''' create joints by setting parent and child relationships and constructing
        the XML formats to be exported later '''

        for k, j in self.joints_dict.items():

            parent = utils.format_urdf_name(j['parent'])
            child = utils.format_urdf_name(j['child'])

            xyz = []
            for p, c in zip(self.links_xyz_dict[parent], self.links_xyz_dict[child]):
                xyz.append(p - c)

            joint = parts.Joint(
                name=k,
                joint_type=j['type'],
                xyz=xyz,
                axis=j['axis'],
                parent=parent,
                child=child,
                upper_limit=j['upper_limit'],
                lower_limit=j['lower_limit']
            )

            self.joints[k] = joint

            # create virtual anchor joint
            if child in self.virtual_links:
                real_link = child.replace('_virtual', '')

                xyz_fixed = []
                for p, c in zip(self.links_xyz_dict[real_link], self.links_xyz_dict[child]):
                    xyz_fixed.append(p - c)

                fixed_joint = parts.Joint(
                    name=f"Virtual_{real_link}_anchor",
                    joint_type="revolute",
                    xyz=xyz_fixed,
                    axis=[1, 0, 0],
                    parent=real_link,
                    child=child,
                    upper_limit=0.001,
                    lower_limit=-0.001,
                )

                self.joints[fixed_joint.name] = fixed_joint
