import typing as tp
import pathlib
from roboticstoolbox.robot.ERobot import ERobot
import numpy as num
import matplotlib.pyplot as plt
from trimesh.scene.scene import Scene
import trimesh
import tempfile
from yourdfpy import URDF


class robotModel(ERobot):
    
    def __init__(self, df: tp.Union[str, pathlib.Path],
                 description_file: tp.Union[str, pathlib.Path]) -> None:

        if isinstance(df, str):
            df = pathlib.Path(df)
        if isinstance(description_file, str):
            description_file = pathlib.Path(description_file)
        df = df.resolve()
        description_file = description_file.resolve()

        # Create rtb robot model
        (eLinks, name, self.urdfStr, filePath) = ERobot.URDF_read(description_file, tld=df)
        super().__init__(eLinks, name=name)

        # Normalise URDF string (ros uses the package:/ syntax, not every python package can read that)
        self.urdfStr = self.urdfStr.replace("package:/", str(df))

        self._visual_urdf = None
        self._visual_meshes = {}
        self._load_visual_urdf()
        self._load_visual_meshes()
        
    def plot_robot(self, q: num.ndarray) -> None:

        # Initialize empty mesh list
        mesh_list = []

        for key in self._visual_meshes.keys():

            # Place mesh using FK
            fkine = self.fkine(q, end=key, start=self.base_link)
            transformed_mesh = self._visual_meshes[key].apply_transform(fkine)
            mesh_list.append(transformed_mesh)
        
        # create a trimesh scene with our geometries in it
        tmsh_scene = Scene(mesh_list)
        
        # Display the scene
        tmsh_scene.show()


    ### Out of role play: Do not feel obliged to read the code bellow. Do it if time, if not skip it
    #   these are just support functions to load stl files
    def _load_visual_urdf(self) -> None:

        # Load the file with yourdfpy (we have to write the urdf into a file to be able to load it)
        with tempfile.TemporaryDirectory() as tmpdirname:
            
            tmp_urdf = pathlib.Path(tmpdirname) / "tmp_urdf.urdf"
            with open(tmp_urdf, "w") as tmpf:
                tmpf.write(self.urdfStr)
            
            self._visual_urdf = URDF.load(tmp_urdf)
            
    def _load_visual_meshes(self) -> None:
        
        if self._visual_urdf is not None:
        
            # Go node by node through the URDF link by link
            for key, link in self._visual_urdf.link_map.items():
                
                # Load all visuals available as trimeshes
                if len(link.visuals) > 0:
                    stl_file = link.visuals[0].geometry.mesh.filename
                    self._visual_meshes[key] = trimesh.load_mesh(stl_file)
                    self._visual_meshes[key] = self._visual_meshes[key].process()
                    # self._visual_meshes[key] = self._visual_meshes[key].smoothed()
                    
                    # Set the link at it's origin point if needed
                    if link.visuals[0].origin is not None:
                        self._visual_meshes[key].apply_transform(link.visuals[0].origin)
        else:
            print("Can't load visual meshes before loading visual URDF")
        
        