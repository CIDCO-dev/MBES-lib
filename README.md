# MBES-lib
Multibeam echosounder function library


## Supported datagram types
* Kongsberg EM series (.all)
* Reson (.s7k)
* Triton (.xtf)

## Sample programs

### cidco-decoder

Decodes binary datagrams to the legacy CIDCO ASCII format


### datagram-dump

Dumps position,attitude and ping data in a single ASCII stream. Lines with position data start with 'P', lines with attitude data start with 'A', and lines with ping data start with 'X'.


### datagram-list

Lists the internal IDs of the packets found inside a binary datagram. Useful for reverse-engineering packet types.


### georeference

Converts a binary file to a 3D point cloud in the WGS84 cartesian frame


