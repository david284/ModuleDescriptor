## top-level section

The top-level of the file contains no single element items. 
Though earlier versions did; v0.6 and earlier. 
From v0.7 onwards all the single element items have been grouped, along with a number of new elements, into logical entities.

The child elements of the top-level section are as follows:
- metaData
- logic
- nodeVariables
- eventVariables

The nodeVariables and eventVariables elements are described in detail in the **nodeVariables & eventVeriables sections**.
The other child elements are described below.

To cater for earlier file formats, if both **metadata** and **logic** are *not* present then the following properties should be attempted to be read from the top-level section:

- moduleDescriptorName
- moduleName
- NVsetNeedsLearnMode

If the above properties are present in both the top-level section *and* either **metadata** or **logic** then those in the top-level section should be ignored.

### metaData

Contains information about the module and the file itself.

- bootable
- consumesOwnEvents
- eventsLength
- eventVariablesLength
- isConsumer
- isProducer
- moduleDescriptorName
- moduleName
- nodeVariablesLength
- transport

### logic

Contains file level logic rules.

- NVsetNeedsLearnMode
- producedEventLogic

#### bootable

A flag property that indicates whether the Module can be reprogrammed via a Bootloader.
Many Modules can be reprogrammed by means of an onboard Bootloader.
This allows a Configuration Tool to be used to reprogram a Module directly; 
and means the user does not need to resort to external hardware.
If this property is not present then its value should be interpreted as *unknown* and the value from the Node Flags used.

Note: it is advised that before attempting to bootload a Module that the Bootable flag from Node Flags should be read to confirm this functionality.

### consumesOwnEvents

A flag property that indicates whether a Module is able to consume its own events.
A Combi Module can both Produce Events and Consume Events.
By default a Module will not consume its own events.
This is because on the default bus (CAN) a Module never sees any message that it sends.
However, by adding any outgoing message to the incoming queue, Module firmware can work around this limitation.
If this property is not present then its value should be interpreted as *unknown* and the value from the Node Flags used.
If **isConsumer** and **isProducer** are not both set to true then this property should be ignored.

### eventsLength

An integer property that indicates the number of Events that the Module supports.
Its range is zero to 255.
If this property is not present then its value should be interpreted as *unknown* and the value from the Module Params used.

### eventVariablesLength

An integer property that indicates the number of Event Variables that the Module supports per Event.
Its range is zero to 255.
This property is only required if the number of supported Event Variables *cannot* be determined from the **eventVariables** property.
If this property is not present then the number of Event Variables should be determined by taking the highest Event Variable index from the **eventVariables** property.

### isConsumer

A flag property that indicates whether the Module is a Consumer of Events.
The Node Flags value as reported by the Module is sometimes incorrect;
this property should correct any misreporting by the Node Flags.
A module can be both a Consumer and a Producer; it is then generally refered to as a *Combi* Module. 
A Combi Module will have both **isConsumer** and **isProducer** flags set.
If this property is not present then its value should be interpreted as *unknown* and the value from the Node Flags used.
However, if the **isProducer** flag is present and this one is not then its value should be interpreted as *false*.

### isProducer

A flag property that indicates whether the Module is a Producer of Events.
The Node Flags value as reported by the Module is sometimes incorrect;
this property should correct any misreporting by the Node Flags.
A module can be both a Consumer and a Producer; it is then generally refered to as a *Combi* Module. 
A Combi Module will have both **isConsumer** and **isProducer** flags set.
If this property is not present then its value should be interpreted as *unknown* and the value from the Node Flags used.
However, if the **isConsumer** flag is present and this one is not then its value should be interpeted as *false*.

### moduleDescriptorName

This string property is the filename described above without the extension. 
This is an important property that is used to identify the descriptor when the content of the file is being 
as a data object programmatically (i.e. not as a file with a filename)

### moduleName

This optional string property is the name registered against the manufacturer ID &
Module ID for this specific module. 
The value returned from the CBUS command NAME is typically a subset of this module name due to data restrictions

### nodeVariablesLength

An integer property that indicates the number of Node Variables that the Module supports.
Its range is zero to 255.
This property is only required if the number of supported Node Variables *cannot* be determined from the **nodeVariables** property.
If this property is not present then the number of Node Variables should be determined by taking the highest Node Variable index from the **nodeVariables** property.

### transport

A string property that provides the name of the transport type the Module supports.
Its values are those defined in CBUSDEFS as CbusBusType.
Typical values are:
- "CAN" for CAN bus.
- "ETH" for Ethernet.
- "MIWI" for Microchip mesh network.
- "USB" for USB.

If this property is not present or contains an invalid or unknown value then its value should be interpreted as *unknown* and the value from the Module Params used.

### NVsetNeedsLearnMode

One family of firmware based on original CANSERVO8 code needs to be put into ‘learn’ mode before node variables can be programmed. 
Setting the NVsetNeedsLearnMode key to a value of true will indicate if this is required for this specific module. 
The processing application is expected to assume false if this property is not present, so it’s only required if set to true

### producedEventLogic

This property holds the logic rule used to determine whether an Event is Produced. 
If it is not Produced then it can be inferred that it is Consumed.

The property uses the structure of a Logic Element.
If the logic resolves to true then the Event is Produced; otherwise it is Consumed.
See the Logic Elements section for further details.

If only one of **isConsumer** and **isProducer** resolve to true,
then it can be inferred that all Events allign with the **isConsumer** / **isProducer** flag values, 
and this property can be optional.
However, if both **isConsumer** and **isProducer** resolve to true then this property is required.
This property is also required if **isConsumer** and or **isProducer** are not present *and* the Node Flags indicate that the Module is a Combi Module.
