## To-do
[x] Add volume control - VCP, VCS, and VOCS
[ ] Add media control (play/pause, skip forward, skip back) - TMAP
[x] Get bluetooth reconnect working so I don't have to delete bond info every time (added bluetooth settings to save bond keys)
[ ] Add microphone support - MICP
[ ] Dual earbud support - CSIP


## HIERARCHY
- A *Profile* is a broad specification that includes one or more *Services*

## List of BT LE Profiles and Services
- Required Profiles
    - GAP: Generic Access Profile - controls connections and advertising
        - GAP makes your device visible to the outside world and determines how devices are allowed to interact with each other
        - Defines roles: Central or Peripheral
        - Two ways to send advertising out: Advertising Data payload and Scan Response payload
            - Advertising Data payload is mandatory, the other is not
            - Limited at 31 bytes each
            - Advertising Data payload is sent out at a specific interval. Longer intervals use less power but feel more responsive
    - GATT: Generic Attribute Profile - defines the way two BLE devices transfer data back and forth using Services and Characteristics
        - Comes into play once a connection has been established via the GAP
        - A Peripheral device is a GATT Server, a Central Device is a GATT Client
        - The Client starts all data transactions
        - When a connection is established, the peripheral suggests a "connection interval" to the central device, which will then try to reconnect every connection interval to see if any new data is available
        - GATT transactions are defined in this nested structure:
            - Profile: Just a predefined/precompiled collection of services
                - Service: Break data up into logical entities containing specific chunks of data called characteristics, can have one or more of them and distinguish themselves from other services using a UUID
                    - Characteristic: Encapsulates a single data point. Distinguished by a UUID as well, and custom Characteristics can be defined. 
    - UUID: Universally Unique Identifiers
- Mesh
    - MESH
    - MMDL: Mesh Model...d...l...(?)
- Health Care Profiles
    - BLP: Blood Pressure Profile
    - HTP: Health Thermometer Profile
    - GLP: Glucose Profile
    - CGMP: Continuous Glucose Monitor Profile
- Sports and Fitness Profile
    - BCS: Body Composition Service
    - CSCP: Cycling Speed and Cadence Profile
    - CPP: Cycling Power Profile
    - HRP: Heart Rate Profile
    - LNP: Location and Navigation Profile
    - RSCP: Running Speed and Cadence Profile
    - WSP: Weight Scale Profile
- Generic Sensors
    - ESP: Environmental Sensing Profile
    - UDS: User Data Service
- HID (Human Interface Device) Connectivity
    - HOGP: HID over GATT Profile
- Proximity Sensing
    - FMP: "Find Me" Profile
    - PXP: Proximity Profile
- Alerts and Time
    - Phone Alert Status Profile
    - Time Profile
- Battery
    - Battery Service
- Audio
    - BAP: Basic Audio Profile
    - CAP: Common Audio Profile
    - PBP: Public Broadcast Profile
    - TMAP: Telephony and Media Audio Profile
    - HAP: Hearing Access Profile
    - BASS: Broadcast Audio Scan Service
    - PACS: Published Audio Capabilities Service
    - Additional Audio Concepts:
        - ASCS: Audio Stream Control Service
        - QoS: Quality of Service
        - ISO: Isochronous Transport
        - CIS: Connected Isochronous Stream
        - BIS: Broadcast Isochronous Stream
        - LC3: Low Complexity Communication Codec
        - ASE: Audio Stream Endpoint