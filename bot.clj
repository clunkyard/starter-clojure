;; A robot on the arena's shared Java core binding.
;; Bearing is earned: a scan contact's bearing is off by up to its
;; bearingError (degrees, a fixed share of the arc), so a wide beam finds a
;; robot and a narrow one aims at it. The robot narrows the beam, holds it
;; on the target and aims along a line fitted through its recent sightings.
;; Contacts within 120 units are exact.
;; Clunkyard robot API 2.0.0: one core under every language, the same
;; answers everywhere. Facts (a call shows its arguments):
;; match: mode ruleset control robots is_3d tick_rate max_ticks
;;   robot_id(slot)
;; clock: tick seconds remaining_ticks
;; arena: size gravity terrain_kind ground_height(x y)
;; me: health position velocity heading pitch roll angular_velocity
;;   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
;;   from_mount(heading pitch) mount_toward(point)
;; battery: charge capacity regen sustainable_drive compute
;;   drive_cost(fraction)
;; drive: hover_lift
;; gun: aim_now cooldown_ticks spread max_power damage_scale
;;   can_fire(power reserve) shot_speed(power) shot_cost(power)
;;   reload_ticks(power) shot_damage(power) power_for_speed(speed)
;;   power_for_flight_time(range seconds) lead(contact power)
;;   lead_point(position velocity power)
;; scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
;; sensors: contacts nearest cues contact(slot) position_of(contact)
;; tracker: cue nearest_bearing_rate velocity(slot) position(slot)
;;   history(slot) last_seen(slot)
;; nav: zone zone.margin(point) zone.contains(point margin)
;;   zone.distance_from(point) storm storm.margin(point)
;;   storm.contains(point margin) storm.distance_from(point) hill
;;   hill.margin(point) hill.contains(point margin)
;;   hill.distance_from(point) pickups nearest_pickup(kind)
;;   field_distance(a b)
;; score: me best armour
;; angles: normalize(degrees) angle_diff(target current)
;;   bearing_to(from to) elevation_to(from to) direction(heading pitch)
;;   distance(a b) to_chassis(heading pitch attitude)
;;   from_chassis(heading pitch attitude) clamp(value low high)
;; raw: number(path) text(path) has(path) flag(path) count(path) line()
;; Controls: drive.set(throttle turn pitch_turn)
;;   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
;;   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
;;   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
;;   status.set(text value) raw.extra(json)
;; Every control argument is optional; a second call keeps what the first
;; set. Absent facts are your language's none. Names are snake_case here;
;; your language cases them its own way. Angles are degrees from the +x
;; axis; pitch is positive up.
;; Manual: https://clunkyard.com/manual#sdk
(import '[clunkyard Robot Clunkyard Control Drive Shape Source Vec3])

(def search-arc 40.0)  ; degrees: sweeps the circle in half a second
(def hold-width 60.0)  ; units across at the target: it stays in the beam from tick to tick
(def min-arc 6.0)      ; degrees: the narrowest the beam holds
(def window 12)        ; ticks of sightings the aim averages
(def sightings 8)      ; narrow sightings in the window before a shot
(def look-every 40)    ; ticks between looks around the target, for a nearer robot
(def reacquire 10)     ; ticks to look again where a lost target was

(defn steer [angles towards current]
  (.clamp ^clunkyard.Angles angles (/ (.angleDiff ^clunkyard.Angles angles (double towards) (double current)) 45)))

;; Dodges taken, the tick the current one ends, and the last target's slot
;; (for looking again where it was lost).
(def dodging (atom {:dodges 0 :end 0 :slot 0}))

;; A straight line through the narrowest of the last `window` ticks' sightings:
;; where the robot is now and how fast it moves. Each sighting is off by its
;; bearingError; the line averages that out.
(defn fit [^Robot bot slot tick]
  (let [recent (filter #(> (.tick ^clunkyard.Sighting %) (- tick window)) (.history (.tracker bot) slot))
        best (when (seq recent) (apply min (map #(.bearingError ^clunkyard.Sighting %) recent)))
        seen (when best (filter #(<= (.bearingError ^clunkyard.Sighting %) (* 1.5 best)) recent)) ; the narrow ones
        n (count seen)]
    (when (>= n 2)
      (let [ticks (map #(.tick ^clunkyard.Sighting %) seen)
            t0 (/ (reduce + ticks) (double n))
            spread (reduce + (map #(Math/pow (- % t0) 2) ticks))
            ;; One axis: the line's position now and its slope per tick.
            axis (fn [of]
                   (let [ps (map #(of (.position ^clunkyard.Sighting %)) seen)
                         mean (/ (reduce + ps) (double n))
                         slope (/ (reduce + (map (fn [t p] (* (- t t0) (- p mean))) ticks ps)) spread)]
                     [(+ mean (* slope (- tick t0))) (* slope (.tickRate (.match bot)))]))
            [x y z] [(axis #(.x ^Vec3 %)) (axis #(.y ^Vec3 %)) (axis #(.z ^Vec3 %))]]
        {:position (Vec3. (first x) (first y) (first z))
         :velocity (Vec3. (second x) (second y) (second z))
         :count n}))))

(defn turn [^Robot bot]
  (let [self (.me bot), position (.position self), angles (.angles bot), gun (.gun bot), scanner (.scanner bot), drive (.drive bot), status (.status bot)
        tick (.tick (.clock bot)), slot (:slot @dodging)
        target (.orElse (.nearest (.sensors bot)) nil), cue (.orElse (.cue (.tracker bot)) nil)
        last-seen (when-not (zero? slot) (let [seen (.lastSeen (.tracker bot) slot)] (when (.isPresent seen) (.getAsInt seen))))
        free (and (.is3d (.match bot)) (zero? (.z (.gravity (.arena bot)))))]
    (.forces drive (Drive/brake 0.2))
    (cond
      target
      (let [^clunkyard.Contact target target
            range (.range target)
            throttle (cond (> range 350) 0.7 (< range 200) -0.4 :else 0.2)
            ;; Halve the beam each tick until it is hold-width across at the target:
            ;; narrow enough for a precise bearing, wide enough to keep it in.
            hold (max min-arc (Math/toDegrees (* 2 (Math/atan2 (/ hold-width 2) range))))
            ;; In a crowd a narrow beam sees one robot. Every look-every ticks,
            ;; widen it for a tick: a nearer robot it finds becomes the target.
            arc (if (zero? (mod tick look-every)) search-arc (max hold (min search-arc (/ (.arc (.last scanner)) 2))))
            power (.powerForFlightTime gun range 1.0)
            ;; An exact contact: .lead is the intercept if the target holds course.
            ;; A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            line (when (pos? (.bearingError target)) (fit bot (.slot target) tick))
            ready (or (zero? (.bearingError target)) (boolean (and line (>= (:count line) sightings))))
            aim (if (pos? (.bearingError target))
                  (when line (.orElse (.leadPoint gun (:position line) (:velocity line) power) nil))
                  (.orElse (.lead gun target power) nil))
            heading (if aim (.heading ^clunkyard.Solution aim) (.bearing target))
            pitch (if aim (.pitch ^clunkyard.Solution aim) (.elevation target))
            now (.aimNow gun)]
        (swap! dodging assoc :slot (.slot target))
        (.set drive throttle (steer angles (.bearing target) (.heading self)) (steer angles (.elevation target) (.pitch self)))
        (.set scanner (.bearing target) arc (.elevation target))
        (.aim gun heading pitch)
        (when (and ready (< (Math/abs (.angleDiff angles heading (.heading now))) 3) (< (Math/abs (- pitch (.pitch now))) 3) (.canFire gun power 10.0)) (.fire gun power))
        (.set status (if ready "TRACK" "NARROW") range))
      ;; Lost it: look again where it was last seen, widening the beam, before searching.
      (and last-seen (<= (- tick last-seen) reacquire))
      (let [where (.orElse (.position (.tracker bot) slot) position)
            bearing (.bearingTo angles position where)
            elevation (if free (.elevationTo angles position where) 0.0)]
        (.set drive 0.5 (steer angles bearing (.heading self)) (steer angles elevation (.pitch self)))
        (.set scanner bearing (min search-arc (* 2 (.arc (.last scanner)))) elevation)
        (.set status "REACQUIRE"))
      cue
      (let [^clunkyard.Cue cue cue, bearing (.bearingTo angles position (.position cue)), distance (.distance angles position (.position cue))
            elevation (if free (.elevationTo angles position (.position cue)) 0.0), error (min (.bearingError cue) 30.0)
            across (* (/ (- (mod tick 5) 2) 2.0) error), up (if free (* (/ (- (mod (quot tick 5) 3) 1) 2.0) error) 0.0)]
        (.set drive 0.7 (steer angles bearing (.heading self)) (steer angles elevation (.pitch self)))
        (.set scanner (+ bearing across) (.arcForReach scanner (+ distance 100)) (+ elevation up))
        (.set status "CUE" distance))
      :else
      (do (.set drive 0.5 0.33 (steer angles 0.0 (.pitch self)))
          (.set scanner (.normalize angles (+ (.heading (.last scanner)) 36)) search-arc (if free (* (- (mod (quot tick 10) 3) 1) 35.0) 0.0))
          (.set status "SEARCH")))
    ;; A simple dodge: vary it, or predict the shooter.
    (when (and (>= tick (:end @dodging)) (some #(= Source/MUZZLE (.source ^clunkyard.Contact %)) (.cues (.sensors bot))))
      (swap! dodging #(-> % (update :dodges inc) (assoc :end (+ tick 10)))))
    (when (< tick (:end @dodging))
      ;; Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
      (let [dodges (:dodges @dodging), left (odd? dodges)]
        (if (= (.control (.match bot)) Control/FORCES)
          (do (.set drive (Drive/throttle 0.5)) (.forces drive (Drive/strafe (if left -1.0 1.0))))
          (.set drive 1.0 (if left 1.0 -1.0)))
        (.set status (if left "dodge:left" "dodge:right") (double dodges))))
    (when (< (.charge (.battery bot)) (/ (.capacity (.battery bot)) 4)) (.set drive (Drive/throttle (* (.orElse (.get drive "throttle") 0.0) 0.5))))
    (let [storm (.orElse (.storm (.nav bot)) nil), zone (.orElse (.zone (.nav bot)) nil)]
      (when (and storm (> (.orElse (.margin ^clunkyard.Storm storm position) -1e9) -60))
        (.set drive (.turn (Drive/throttle 0.8) (.clamp angles (/ (.angleDiff angles (.bearingTo angles (.center ^clunkyard.Storm storm) position) (.heading self)) 30)))))
      (when (and zone (< (.orElse (.margin ^clunkyard.Zone zone position) 1e9) 60))
        (let [climb (if (= (.shape ^clunkyard.Zone zone) Shape/SPHERE) (.elevationTo angles position (.center ^clunkyard.Zone zone)) 0.0)]
          (.set drive 0.8 (.clamp angles (/ (.angleDiff angles (.bearingTo angles position (.center ^clunkyard.Zone zone)) (.heading self)) 30)) (.clamp angles (/ (- climb (.pitch self)) 30))))))
    ))

(let [robot (atom nil), bot (proxy [Robot] [] (onTurn [] (turn @robot)))]
  (reset! robot bot)
  (Clunkyard/run bot))
