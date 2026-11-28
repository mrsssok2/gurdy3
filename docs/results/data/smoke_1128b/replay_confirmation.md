5 fetch tries with a sighting log: can_1/can_1, can_2/can_1, can_3/can_1, cube/small_box_1, can_1/can_1

| setting | target confirmed | confirmed tracks (sum) | false tracks (sum) | per try: target / false |
|---|---|---|---|---|
| node default (8 hits, span 2 s, ttl 10 s) | 4/5 | 25 | 3 | can_1/can_1 Y/0; can_2/can_1 Y/0; can_3/can_1 Y/1; cube/small_box_1 n/2; can_1/can_1 Y/0 |
| ttl 30 s | 5/5 | 31 | 3 | can_1/can_1 Y/0; can_2/can_1 Y/0; can_3/can_1 Y/1; cube/small_box_1 Y/2; can_1/can_1 Y/0 |
| ttl 60 s | 5/5 | 31 | 3 | can_1/can_1 Y/0; can_2/can_1 Y/0; can_3/can_1 Y/1; cube/small_box_1 Y/2; can_1/can_1 Y/0 |
| ttl 120 s | 5/5 | 31 | 3 | can_1/can_1 Y/0; can_2/can_1 Y/0; can_3/can_1 Y/1; cube/small_box_1 Y/2; can_1/can_1 Y/0 |
| 5 hits, ttl 10 s | 5/5 | 37 | 5 | can_1/can_1 Y/1; can_2/can_1 Y/1; can_3/can_1 Y/1; cube/small_box_1 Y/2; can_1/can_1 Y/0 |
