# Working with Julia notes

I've been working on this rather disjointedly, and I feel like I keep forgetting everything I learn about Julia from the last time I worked on it. Here's to hoping this will help me keep better track of what I'm doing. 

## Coordinate frames
Each rigid body has a default coordinate frame, which can be accessed with ```default_frame(body)```. 

To visualize frames as a triad in MeshCat, use ```setelement!(mvis, frame)```, which in my case would be ```setelement!(mvis_alpha, base_frame)```.

Visualizations are through MeshCatMechanisms. The github is here: https://github.com/JuliaRobotics/MeshCatMechanisms.jl

## File navigation
You can find out where packages are stored with ```pathof(packagename)```


## Running issues

10/12: can't use ```frame_definitions(body)``` or ```is_fixed_to_body(body, frame)```, which I think is what's causing the error on ```transform(state, wrench, frame)``` 
```
UndefVarError: [91mUndefVarError: frame_definitions not defined[39m
UndefVarError: frame_definitions not defined

Stacktrace:
 [1] top-level scope at In[84]:7
 ```

10/13: problem magically resolved itself after restart -_-

**ACTUALLY** problem resolved because the Julia session has to end in order to reimport the packages. So my changes to RigidBodyDynamics.jl weren't being applied.

Going down a rabbit hole of trying to get GeometryBasics installed so I can make arrows. 
```
(v1.0) pkg> add GeometryBasics
  Updating registry at `~/.julia/registries/General`
  Updating git-repo `https://github.com/JuliaRegistries/General.git`
 Resolving package versions...
ERROR: Unsatisfiable requirements detected for package GeometryBasics [5c1252a2]:
 GeometryBasics [5c1252a2] log:
 ├─possible versions are: [0.1.0-0.1.3, 0.2.0-0.2.15, 0.3.0-0.3.13, 0.4.0-0.4.1] or uninstalled
 ├─restricted to versions * by an explicit requirement, leaving only versions [0.1.0-0.1.3, 0.2.0-0.2.15, 0.3.0-0.3.13, 0.4.0-0.4.1]
 └─restricted by julia compatibility requirements to versions: uninstalled — no versions left
 ```
 
Updated to Julia 1.6.3 instead of 1.0.4 and now GeometryBasics is installed. It took approximately an hour and a half to upgrade Julia and get the jupyter notebook working again. Really, all you need to do it install Julia again, change the ~/.bashrc file to add the right installation to the path, delete the old one from the bashrc, and enjoy. You will have to restart Julia, use the package manager to add IJulia again, and build IJulia. (Or you might have to manually make another jupyter kernel by copying the existing one and changing its name... but hopefully it wasn't that.) Restart the terminal or VS Code often to make sure the things you're doing are taking effect. 

10/14: Added frames for center of buoyancy and made wrenches in those locations. Finally figured out how to pass that dictionary of wrenches around. Next steps: add buoyancy to the inverse dynamics; add the added mass and coriolis terms; add one degree of freedom in world space. 

10/20: Controller now calls hydro_calc! and uses it as an input to inversedynamics(). Kind of assumes we have perfect knowledge of the state of the robot. 

Started trying to implement added mass forces, but it's creating NaNs in simulation. Maybe try converting the torques into external wrenches instead.

That also led to instabilities

10/21: New plan: add M_A and C_A as extra mass in URDF and implement buoyancy and gravity as external forces. This seems to work better. New script: alphasim_comboURDF.ipynb and new URDF: alphaArmCombo.urdf