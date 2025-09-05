# EEC 195 Senior Design Project

## TODO:

-   [ ] Normalize errors (from -1 to 1 ideally, then scale proportionally when determining steering values)
-   [ ] Control speed based on error: Closer to 0 = More Speed, Closer to 1/-1 = Less Speed
-   [ ] Fine-tune PID values, error weights (i.e. line-center error vs. angle error)
    -   [Good guide](https://thecodingfun.com/2020/06/16/lego-mindstorms-ev3-pid-line-follower-code-by-using-micropython-2-0/) on how to determine values
