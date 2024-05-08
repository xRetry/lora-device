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
