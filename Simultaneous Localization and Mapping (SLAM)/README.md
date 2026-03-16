Venkat Suryaansh Thota 
25b3952	
Data Association:
The baseline allowed two measurements to claim the same cone at the same time. The Hungarian algorithm, which finds the best one-to-one matching globally in a single shot, took its place. Additionally, a gate was added that rejects measurements that are too far from any cone to be a reasonable match; instead of being force-matched to something incorrect, these measurements are marked as outliers

Localization:
The baseline never looked at the sensor; it just performed physics. As a result, every frame was exacerbated by a tiny heading error, and the position gradually deviated from reality. An EKF was added that performs two tasks at each stage: first, it uses the same physics to predict the new position; next, it examines the cones the sensor can see, compares them to where they should be given the current position estimate, and uses the difference to push the estimate closer to reality. It corrects more forcefully when the position estimate is more ambiguous.

Mapping:
The baseline stored the first noisy sighting of each cone forever and never updated it. Replaced it with a running average — every time the same cone is seen again, the position estimate moves slightly toward the new measurement. Also added a confirmation rule so a cone only enters the official map after being seen 3 times, which stops ghost detections from polluting the map permanently. Tentative cones that disappear quickly get deleted.

