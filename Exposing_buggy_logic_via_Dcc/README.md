# Exposing buggy logic via Dcc

During the evaluation, we find that Dcc can also be used to identify buggy logic in the swarm algorithm, even when Swarmbug cannot find a fix to the target bug.

When we initially evaluate Howard's algorithm, we find that Swarmbug could not find any possible fix.
We investigate the Dcc values produced during the experiment further and find out that the observed Dcc values are extremely stable, except for slight variations observed in `obst_pot_c` just before the drone crashes.

As we trace back to code related to `obst_pot_c`, we found that there is no code that calculates the potential field outside of obstacles, meaning that the drone will only start to detect obstacles after they already crash.
To this end, we modify the algorithm so that it can detect objects early.
After we patch the algorithm, we conduct our evaluation on the algorithm again, and Swarmbug successfully finds a possible fix.

Details are as follows.

## Trend of Dcc value

### Normal version (no patch)

![](https://github.com/swarmbug/src/blob/main/Exposing_buggy_logic_via_Dcc/data/Dcc%20trend%20of%20patched%20version%20normal.png)

- x-axis: simulation tick
- y-axis: Dcc value
- For readibility this example is extracted from experiments using only 4 drones. (The experiments in paper was done with 20 drones.)

Above figure indicates drone 1's trend of Dcc value around the obstacle to avoid (tick: 36-42).
We can find that obstacle's dcc value is increasing from 37 tick and decreasing after 40, which is the time drone 1 is approaching to the drone and passing it without avoiding.
Also, except for obstacle, majority area is waypoint and other drones dcc values are minor.
It means that, before drone 1 meets the obstacle, the other drones' influence is little, and waypoint is largely contributing to drone 1's decision making.
We take a Reinforcing (increasing) strategy here.

### Patched version with '`d_pot_out_c` x 2.0'

![](https://github.com/swarmbug/src/blob/main/Exposing_buggy_logic_via_Dcc/data/Dcc%20trend%20of%20patched%20version%20d_pot_out_c%20x%202.0.png)

This is the trend of dcc value of same situation with patch '`d_pot_out_c` x 2.0'.
We cannot find any significant change in any objectives.
Of course drones cannot avoid obstacles with this patch.
This is similar in cases of other pathces.

### Patched version with '`obst_pot_c` x 2.0'

![](https://github.com/swarmbug/src/blob/main/Exposing_buggy_logic_via_Dcc/data/Dcc%20trend%20of%20patched%20version%20obst_pot_c%20x%202.0.png)

This is the trend of dcc value of same situation with patch '`obst_pot_c` x 2.0'.
Although drones cannot avoid obstacles with this patch, the figure shows clearly that the area of obstacle's dcc value is increased as we intend using Reinforcing (increasing) strategy.
When we increase the value of this parameter up to 'x 10.0', we observe Dcc value changes but drone still crashes with obstacle.
Then we decide to look into more the code around this parameter.

## Before fix

### Problematic code

Below code is from \*obstacle.m". We can find `c_obst_pot_c` is used here.
Here, `obj` indicates obstacle (class) and `r` indicates the distance between an obstacle and the drone. When r >= R, `val` takes 0 so that an agent (drone) cannot catch any value outside of obstacle's radius.
Currently, all patch strategies does not work with this code.

```matlab=
methods (Access = private)

    % Get potential caused by obstacle
    function val = GetObstaclePotential( obj, r  )
        % Create a potential function that increases for r >
        % obj.radius and for r <= obj.radius such that r = obj.radius
        % becomes a minimum
            R = obj.radius;
            obst_pot_c = obj.c_obst_pot_c * 1e3;
            if( r < R )
                val = obst_pot_c *exp( -(r/R)^2 );
            else
                % Problematic spot
                val = 0;
            end

    end% End center drone potential function

% End private methods
end

```

### Video of current version

In this video, all objects are placed in 3-dimension space.

- green sphere: agent (drones)
- blue sphere: waypoint
- red sphere: obstacle

The first obstacle is placed on the route between the first and second waypoint.
In this version of video, drones (green sphere) do not avoid the first obstacle (red sphere).

![](https://github.com/swarmbug/src/blob/main/Exposing_buggy_logic_via_Dcc/videos/no_fix.gif)

## After fix

### Fixed code

```matlab=
methods (Access = private)

    % Get potential caused by obstacle
    function val = GetObstaclePotential( obj, r  )
        % Create a potential function that increases for r >
        % obj.radius and for r <= obj.radius such that r = obj.radius
        % becomes a minimum
            R = obj.radius;
            obst_pot_c = obj.c_obst_pot_c * 1e3;
            if( r < R )
                val = obst_pot_c *exp( -(r/R)^2 );
            else
                % after fix
                val = obst_pot_c *exp( -(r/R)^2 );
            end

    end% End center drone potential function

% End private methods
end

```

### Video of patched version

This video is the patched version (with `obj.c_obst_pot_c` x 8.0) with the fixed code.
As you can see below video, each drone (green sphere) recognizes and avoids the obstacle (red sphere).

![](https://github.com/swarmbug/src/blob/main/Exposing_buggy_logic_via_Dcc/videos/fix.gif)
