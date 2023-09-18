# FleetCommand
A system for networking space engineers vessels together. 

The fleet command protocol will automatically discover other crafts and negotiate how to link up with them. 
By default all vessels will eventually form one large network. If two ship networks become visible to eachother, the default behaviour is to merge those two networks (and their information) into one.
The networking layer running fleet command supports serialization/deserialization of any data type that implements ISerializable. Encryption or compression is also supported via streams.
While the fleet command usually only handles network management and a couple of built-in features, custom extension modules can be baked in to allow user defined code totap into the network.

## Why is this useful?
Fleet command allows for exchange of meaningful data between vessels and can operate even as some members of the network are destroyed or become unavailable. You could use this system for the following scenarios:
- Exchanging IFFs: Fleet command can be used to allow code to understand where the fleet is and what it is doing.
- Cooperative target tracking with lidar or other means: Fleet-wide tracking data is useful for craft using custom missiles
- Exchanging targeting data between ships: Long range targeting data allows for long range weapons to target enemy vessels without putting friendly ships at risk
- Managing and operating AI craft: The fleet command protocol allows for using custom message types and extension modules. You can combine these to implement logic that receives custom telemetry from AI craft or commands them over the network.
- Exchanging navigation/docking data: Useful for autopilots and automatic docking.

## Progress
### FleetCommand.Protocol
- Vessel announcement and discovery: ❌
- Network announcement and discovery: ❌
- Network join: ❌
- Network leave: ❌
- Network leader negotiation: ❌
- Network member merge: ❌
- Network data merge: ❌

### FleetCommand.Networking
- Message reader: ✔️
- Message sender: ✔️
- Public read/write: ✔️
- Network read/write: ✔️
- Cross-network read/write: ✔️
- Message handlers: ✔️
- Message handler options: ✔️

### FleetCommand.IO
- Streams: ✔️
- In-memory buffer stream: ✔️
- Binary serialization: ✔️
- Binary deserialization: ✔️

### FleetCommand.Cryptography
- Encrypted stream: ✔️
- XOR crypto provider (largely unsafe but efficient): ✔️
