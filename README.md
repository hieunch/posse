
## Posse
A toolbox for developing, debugging and evaluating routing algorithms in wireless sensor networks, especially for those routing algorithms dealing with the presence of holes.

## Components

The system consists of 3 parts: 
* Castalia (forked from [Castalia](https://github.com/boulis/Castalia/)) as a simulator for Wireless Sensor Networks. As described
in the Castalia repo: "Castalia is a simulator for WSNs based on the OMNeT++ platform and can be used by researchers and developers who want to test their distributed algorithms and/or protocols in realistic wireless channel and radio models, with a realistic node behaviour especially relating to access of the radio".
* A web app acts as a tool to facilitate the process of generating sensor networks, 
creating holes, selecting regions of communication, sending simulation requests and displaying results
* Nodejs server act as a bridge from client and the simulator. Server receives simulation requests in the JSON format, transform the request into appropriate .ned Castalia network file, invoke the simulation, parse the result and send back to the client.

In this repo, some features are added to Castalia to support various types of 
logging, drawing and statistics aggregating. Several well-known routing algorithms in wireless sensor network dealing with the presence
of holes in literature are also implemented.

Here is basically how the system works:
![Alt text](images/system_architecture.png?raw=true "System architecture")

## Implemented routing algorithms
- byPassRouting and multipathRingsRouting, 2 default routing algorithm of Castalia
- greedyRouting - Greedy routing 
- gpsrRouting - The famous Greedy Perimeter Stateless Routing ([GPSR](http://www.icir.org/bkarp/jobs/gpsr-mobicom2000.pdf)) propsed by Brad Karp and H. T. Kung
- rollingBallRouting - Greedy Anti-void Routing ([GAR](https://ieeexplore.ieee.org/document/4273761/)) proposed by W. J. Liu and K. T. Feng using the idea of a rolling ball.
- shortestPathRouting - Shortest Geometric Path Routing, route the packet along side the shortest geometric path which doesn't intersect routing holes.
- stableRouting - Stable Routing, a routing protocol proposed by members of SEDIC Labs (including the author of the repo) to solve the problem of balancing traffics on the holes boundary and lengthening network lifetime.

The source codes of all routing algorithms 
are located in `Castalia/Castalia/src/node/communication/routing` directory.

## Drawing and logging APIs

Belows are added APIs to Castalia to support drawing and debugging when developing new routing algorithms, 
use a method in drawing APIs will yield the equivalent drawing on client's app after submitting and receiving simulation result.

	void debugLine(Point, Point, string color);
  Draw a line segment between two point with given color
	
	void debugCircle(Point, double, string color);
  Draw a circle with given center, radius and color
	
	void debugPoint(Point, string color);
  Draw a point in a given location and color
	
	void debugPolygon(vector<Point>, string color);
  Draw a polygon given vertices and color

	void debugPath(vector<Point>, string color);
  Draw a geometric path given vertices and color
	
	void debugArc(Point from, Point to, double radius, string color);
  Draw an circle arc from one point to another given radius and color

Example of using drawing APIs:
![Alt text](images/debug_apis.png?raw=true "System architecture")

## Using Client
First step is to create a network
![Alt text](images/demo1.png?raw=true "System architecture")

There are two modes, one is the debug mode, in which we will send 
simulation information and get the drawing and logging info (from drawing APIs above)
from the algorithm we are developing and display on the screen.

Specify pairs of source and destination. Since we're in debug mode, 
each node will be sending only 1 packet
![Alt text](images/demo2.png?raw=true "System architecture")
Get the drawing info back and render
![Alt text](images/demo3.png?raw=true "System architecture")


## Installing


