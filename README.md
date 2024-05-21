## Debian Setup

The Debian installation follows the installer (non-desktop version).
On the screen for software selection, only `SSH server` and `standard system utilities` are chosen

Once the installer is finished, execute the admin setup script to create the `config` user with the default password `pw`.

```sh
sh user_setup.sh
```

The second script needs to be run as this `config` user.

```sh
sudo sh server_setup.sh
```

It is recommended to change the default password of the `config` user with:

```sh
passwd
```

## The Things Network Setup

### Gateway Registration

1. `Gateways` → `Register gateway`
2. Fill out `Gateway EUI` (printed on the gateway next to `(92)`)
3. Fill out `Gateway ID`
4. Fill out `authentication code` (Wi-Fi PW printed on gateway)
5. Select Frequency plan: `Europe 863-870 MHz (SF9 for RX2 - recommended)`
6. `Claim gateway`

### Application Creation

1. `Applications` → `Create application`
2. Fill out `Application ID`
3. Fill out `Application name`
4. `Register end device`
5. `Enter end device specifics manually`
6. Select Frequency plan: `Europe 863-870 MHz (SF9 for RX2 - recommended)`
7. Select LoRaWAN version: `LoRaWAN Specification 1.0.0`
8. `Show advanced activation`
9. Activation mode: `Activation by personalization (ABP)`
10. Additional LoRaWAN class capabilities: `None (class A only)`
11. Network defaults: Check `Use network's default MAC settings`
12. Generate DevEUI
13. Generate Device address
14. Generate AppSKey (application session key)
15. Generate NwkSKey (network session key)
16. Click `Advanced MAC settings` _ check `Reset frame counters`
17. `Register end device`

The following payload formatter converts the bytes of the message to decimal values:

```javascript
function Decoder(bytes, port) {
    const toSigned = (x) => (x > 0x7FFF) ? x - 0x10000 : x;
    return {
        temperature: toSigned((bytes[0] << 8) + bytes[1]) / 1000,
        humidity: ((bytes[2] << 8) + bytes[3]) / 1000
    };
}
```



Trying to commit