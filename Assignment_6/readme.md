## Instruction

### keyboard group

#### Insert , translation and delete

- ``i``: start insert, the insertion will make group as triple click and create a triangle. It is possible to create various number of triangle at insert state.

- ``o``: go to translation mode and allow moving, rotating, scalling.

  after press ``o``, the following key-press are active: 

  - ``h``: rotate 10 degree clockwise
  - ``j``: rotate -10 degree clockwise
  - ``k``: scale 25% up
  - ``l``: scale -25% down

- ``p``: delete the selected triangle.

#### camera

- ``a`` : translate the entire scene 20% right

- ``d`` : translate the entire scene 20% left
- ``w`` : translate the entire scene 20% down
- ``s`` : translate the entire scene 20% up
- ``-`` : translate the entire scene 20% shrink
- ``=`` : translate the entire scene 20% expand

#### color

- ``c``:  go to color mode, set the vertex selection active.

- ``1-9``: color map in ``ColorMap[1:9] ``

  here is the color map: 

  ```c++
  void ColorInit(){
      ColorMap[0] << 0,0,0,1; //black
      ColorMap[1] << 1,0,0,1;
      ColorMap[2] << 0,1,0,1;
      ColorMap[3] << 0,0,1,1;
      ColorMap[4] << 1,1,0,1;
      ColorMap[5] << 0,1,1,1;
      ColorMap[6] << 1,0,1,1;
      ColorMap[7] << 1,0.5,0,1;
      ColorMap[8] << 1,0,0.5,1;
      ColorMap[9] << 0.5,1,0.5,1;
  }
  ```

  

#### keyframe

- ``v`` : start the keyframe mode or insert a keyframe (of current window)
- ``n`` : play the animation in linear interpolation method.
- ``b`` : play the animation in 2-order bezier method. **(Note! Specific three keyframe are needed)**
- ``m`` : cancel all keyframe and quit, go to the state the same as 1st keyframe.

#### control

- `` ``(space): go to norm state, set all triangle inactive.
- ``]`` : dump the current state at anytime, without change state.



