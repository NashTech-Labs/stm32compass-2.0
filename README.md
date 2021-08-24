This template is an **Upgrade version of Compass** as using this we can measure the direction(N, S, W, E, NS etc) of Magnetic Field based
on theta angle.
This template also **Calculate the Magnitude** to the Magnetic Field by taking x, y, z axis values.

This template covers the [Take 2](https://docs.rust-embedded.org/discovery/15-led-compass/take-2.html) Challenge of The Discovery Book using lsm303agr. 

## What I am using here to complete the challenge.
- **Rust Programming Language.**
- **A stm32f3 Discovery Board.**
- **Few dependencies.**

## What we are actually doing here?

We are going to provide the x, y, z axis values and then we are calculating the theta and according to that we are 
checking the direction.

We are also printing the Magnitude of the Magnetometer.

To-Verify you need to move the board and change the direction to read different (x,y,z)axis values. 

You will find out:

**For theta Value:**

**0 - 0.3925 => South**

**0.3925 - 1.1775 => SouthEast**

**1.1775 - 1.9625 => East**

**1.9625 - 2.7475 => NorthEast and so on..**



## Build the Project
**Now before building this project you need to solder your board. It will help in printing the data to itm terminal.
Use this [link](https://docs.rust-embedded.org/discovery/06-hello-world/index.html) to solder your f3 Board.

#### Note: Make sure the F3 Board is connected to your computer.

### Step 1:
- Open terminal from **home directory** and execute Command

`cd /tmp && touch itm.txt`

Then

`itmdump -F -f itm.txt`

Leave this terminal running. Now in new terminal run command.

`cd /tmp && openocd -f interface/stlink-v2-1.cfg -f target/stm32f3x.cfg`

### Step 2:
Execute the Command

`cargo run`

from directory `compass2.0`

Then we will be in gdb terminal. Now execute command:

- `return`
- `return`
- `step`
- `continue`
