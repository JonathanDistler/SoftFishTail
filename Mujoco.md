<!-- Uses preexisting tendonfishsimple model to develop a new fish issues come from not being able to rotate the meshes appropriately-->
<!-- Try to figure out how to get the meshes, add a motor? (look at Mike's docs), then also mess around with some of the weight parametersand some of the string setups-->
<mujoco model="tendonFish">
    <option integrator="implicitfast" timestep="0.01" iterations="50" solver="Newton"
            tolerance="1e-10" gravity="0 0 0" density="1000" viscosity="0.001" />

    <visual>
        <global offheight="2160" offwidth="3840" />
        <rgba haze=".3 .3 .3 1" />
    </visual>

    <!-- Environment parameters -->
    <asset>
        <mesh name="tailMesh" file="mujoco-3.3.2-windows-x86_64/model/mesh/tailV1.STL" scale="0.0025 0.0025 0.0025" />
        <mesh name="headMesh" file="mujoco-3.3.2-windows-x86_64/model/mesh/headExternal.STL" scale="0.0025 0.0025 0.0025" />
        <texture type="skybox" builtin="gradient" rgb1="0.6 0.6 0.6" rgb2="0 0 0" width="512" height="512" />
        <texture name="texplane" type="2d" builtin="checker" rgb1=".25 .25 .25" rgb2=".3 .3 .3"
                 width="512" height="512" mark="cross" markrgb=".8 .8 .8" />
        <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="1 1" texuniform="true" />
    </asset>

    <default>
        <joint type="hinge" pos="0 0 0" axis="0 0 1" range="-80 80" stiffness="1" damping="0.1" />
    </default>

    <!-- World body and fish structure -->
    <worldbody>
        <camera name="fixed" pos="0.4 -1.2 1.2" xyaxes="1 0 0 0 1 1" />
        <camera name="fixedTop" pos="-0.75 0 1.5" xyaxes="0 -1 0 1 0 0.75" />
        <camera name="fixedDiag" pos="2.5 -4.0 4.0" xyaxes="0.8 0.6 0 -0.4 0.5 0.75" />
        <geom name="floor" pos="0 0 -0.5" size="0 0 1" type="plane" material="matplane" />
        <light directional="true" diffuse=".8 .8 .8" specular=".2 .2 .2" pos="0 0 5" dir="0 0 -1" />

        <body pos="0 0 0">
            <freejoint />
            <geom name="headMesh" type="mesh" mesh="headMesh" pos="0 0 0" euler="0 1.5705 0"
                  rgba=".9 .9 .5 1" mass=".1" />
            <geom name="headSegment" type="box" pos="0.3 0 0" size="0.1 0.01 0.1"
                  rgba=".9 .9 .9 1" mass="0.1" />
            <geom name="headLeft" type="box" pos="0.3 -0.075 0" size="0.01 0.075 0.05"
                  rgba=".9 .9 .9 1" mass="0.1" />
            <geom name="headRight" type="box" pos="0.3 0.075 0" size="0.01 0.075 0.05"
                  rgba=".9 .9 .9 1" mass="0.1" />
            <site name="headLeft" pos="0.3 -0.15 0" size="0.02" rgba=".5 .9 .5 1" />
            <site name="headRight" pos="0.3 0.15 0" size="0.02" rgba=".5 .9 .5 1" />

            <!-- Tail segments -->
            <body pos="0.4 0 0">
                <geom name="headJoint" type="cylinder" pos="0 0 0" size="0.04 0.11" rgba=".6 .1 .1 1" mass="0" />
                <joint name="headJoint" />
                <geom name="tail0" type="box" pos="0.1 0 0" size="0.1 0.01 0.1" rgba=".5 .5 .9 1" mass="0.1" />
                <geom name="tail0left" type="box" pos="0.1 -0.075 0" size="0.01 0.075 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                <geom name="tail0right" type="box" pos="0.1 0.075 0" size="0.01 0.075 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                <site name="tail0left" pos="0.1 -0.15 0" size="0.02" rgba=".5 .9 .5 1" />
                <site name="tail0right" pos="0.1 0.15 0" size="0.02" rgba=".5 .9 .5 1" />

                <body pos="0.2 0 0">
                    <geom name="joint01" type="cylinder" pos="0 0 0" size="0.04 0.11" rgba=".6 .1 .1 1" mass="0" />
                    <joint name="joint01" />
                    <geom name="tail1" type="box" pos="0.1 0 0" size="0.1 0.01 0.1" rgba=".5 .5 .8 1" mass="0.1" />
                    <geom name="tail1left" type="box" pos="0.1 -0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                    <geom name="tail1right" type="box" pos="0.1 0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                    <site name="tail1left" pos="0.1 -0.1 0" size="0.02" rgba=".5 .9 .5 1" />
                    <site name="tail1right" pos="0.1 0.1 0" size="0.02" rgba=".5 .9 .5 1" />

                    <body pos="0.2 0 0">
                        <geom name="joint12" type="cylinder" pos="0 0 0" size="0.04 0.11" rgba=".6 .1 .1 1" mass="0" />
                        <joint name="joint12" />
                        <geom name="tail2" type="box" pos="0.1 0 0" size="0.1 0.01 0.1" rgba=".5 .5 .8 1" mass="0.1" />
                        <geom name="tail2left" type="box" pos="0.1 -0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                        <geom name="tail2right" type="box" pos="0.1 0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                        <site name="tail2left" pos="0.1 -0.1 0" size="0.02" rgba=".5 .9 .5 1" />
                        <site name="tail2right" pos="0.1 0.1 0" size="0.02" rgba=".5 .9 .5 1" />

                        <body pos="0.2 0 0">
                            <geom name="joint23" type="cylinder" pos="0 0 0" size="0.04 0.11" rgba=".8 .3 .3 1" mass="0" />
                            <joint name="joint23" />
                            <geom name="tail3" type="box" pos="0.1 0 0" size="0.1 0.01 0.1" rgba=".5 .5 .8 1" mass="0.1" />
                            <geom name="tail3left" type="box" pos="0.1 -0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                            <geom name="tail3right" type="box" pos="0.1 0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                            <site name="tail3left" pos="0.1 -0.1 0" size="0.02" rgba=".5 .9 .5 1" />
                            <site name="tail3right" pos="0.1 0.1 0" size="0.02" rgba=".5 .9 .5 1" />

                            <body pos="0.2 0 0">
                                <geom name="joint34" type="cylinder" pos="0 0 0" size="0.04 0.11" rgba=".8 .3 .3 1" mass="0" />
                                <joint name="joint34" />
                                <geom name="tail4" type="box" pos="0.1 0 0" size="0.1 0.01 0.1" rgba=".5 .5 .8 1" mass="0.1" />
                                <geom name="tail4left" type="box" pos="0.1 -0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                                <geom name="tail4right" type="box" pos="0.1 0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                                <site name="tail4left" pos="0.1 -0.1 0" size="0.02" rgba=".5 .9 .5 1" />
                                <site name="tail4right" pos="0.1 0.1 0" size="0.02" rgba=".5 .9 .5 1" />

                                <body pos="0.2 0 0">
                                    <geom name="joint45" type="cylinder" pos="0 0 0" size="0.04 0.11" rgba=".8 .3 .3 1" mass="0" />
                                    <joint name="joint45" />
                                    <geom name="tail5" type="box" pos="0.1 0 0" size="0.1 0.01 0.1" rgba=".5 .5 .8 1" mass="0.1" />
                                    <geom name="tail5left" type="box" pos="0.1 -0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                                    <geom name="tail5right" type="box" pos="0.1 0.05 0" size="0.01 0.05 0.05" rgba=".5 .5 .9 1" mass="0.1" />
                                    <site name="tail5left" pos="0.1 -0.1 0" size="0.02" rgba=".5 .9 .5 1" />
                                    <site name="tail5right" pos="0.1 0.1 0" size="0.02" rgba=".5 .9 .5 1" />

                                    <body pos="0.2 0 0" euler="0 0 0">
                                        <geom name="tailJoint" type="mesh" mesh="tailMesh" pos="0 0 0" rgba=".9 .9 .5 1" mass="0" />
                                        <joint name="tailJoint" />
                                        <geom name="tail" type="box" pos="0.05 0 0" size="0.05 0.01 0.1" rgba=".9 .9 .9 1" mass="0.1" />
                                        <geom name="tailFin" type="box" pos="0.22 0 0" size="0.12 0.01 0.2" rgba=".9 .9 .9 1" mass="0.2" fluidshape="ellipsoid" />
                                        <geom name="tailLeft" type="box" pos="0.1 -0.025 0" size="0.01 0.025 0.05" rgba=".9 .9 .9 1" mass="0.1" />
                                        <geom name="tailRight" type="box" pos="0.1 0.025 0" size="0.01 0.025 0.05" rgba=".9 .9 .9 1" mass="0.1" />
                                        <site name="tailLeft" pos="0.1 -0.05 0" size="0.02" rgba=".5 .9 .5 1" />
                                        <site name="tailRight" pos="0.1 0.05 0" size="0.02" rgba=".5 .9 .5 1" />
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <tendon>
        <spatial name="tendonLeft" width="0.01" rgba=".5 .9 .5 1" stiffness="1" damping="0.5">
            <site site="headLeft" />
            <site site="tail0left" />
            <site site="tail1left" />
            <site site="tail2right" />
            <site site="tail3left" />
            <site site="tail4right" />
            <site site="tail5right" />
            <site site="tailRight" />
        </spatial>
        <spatial name="tendonRight" width="0.01" rgba=".5 .9 .5 1" stiffness="1" damping="0.5">
            <site site="headRight" />
            <site site="tail0right" />
            <site site="tail1right" />
            <site site="tail2left" />
            <site site="tail3right" />
            <site site="tail4left" />
            <site site="tail5left" />
            <site site="tailLeft" />
        </spatial>
    </tendon>

    <actuator>
        <position name="muscleLeft" tendon="tendonLeft" ctrlrange="0.4 1.5" kp="100" kv="10" />
        <position name="muscleRight" tendon="tendonRight" ctrlrange="0.4 1.5" kp="100" kv="10" />
    </actuator>
</mujoco>
