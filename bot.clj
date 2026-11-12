;; A robot on the arena's shared Java core binding.
;; Clunkyard robot API 2.0.0: one core under every language, the same
;; answers everywhere. Facts (a call shows its arguments):
;; match: mode ruleset control robots is_3d tick_rate max_ticks
;;   robot_id(slot)
;; clock: tick seconds remaining_ticks
;; arena: size gravity terrain_kind ground_height(x y)
;; me: health position velocity heading pitch roll angular_velocity
;;   attitude grounded altitude applied
;; battery: charge capacity regen sustainable_drive compute
;;   drive_cost(fraction)
;; drive: hover_lift
;; gun: aim_now cooldown_ticks spread max_power damage_scale
;;   can_fire(power reserve) shot_speed(power) shot_cost(power)
;;   reload_ticks(power) shot_damage(power) power_for_speed(speed)
;;   power_for_flight_time(range seconds) lead(contact power)
;;   lead_point(position velocity power)
;; scanner: last proximity_range reach(arc) arc_for_reach(range)
;; sensors: contacts nearest cues contact(slot) position_of(contact)
;; tracker: cue nearest_bearing_rate velocity(slot) position(slot)
;;   last_seen(slot)
;; nav: zone zone.margin(point) zone.contains(point margin)
;;   zone.distance_from(point) storm storm.margin(point)
;;   storm.contains(point margin) storm.distance_from(point) hill
;;   hill.margin(point) hill.contains(point margin)
;;   hill.distance_from(point) pickups nearest_pickup(kind)
;;   field_distance(a b)
;; score: me best armour
;; angles: normalize(degrees) angle_diff(target current)
;;   bearing_to(from to) elevation_to(from to) direction(heading pitch)
;;   distance(a b) clamp(value low high)
;; raw: number(path) text(path) has(path) flag(path) count(path) line()
;; Controls: drive.set(throttle turn pitch_turn)
;;   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
;;   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
;;   tracker.configure(smoothing forget_ticks cue_ticks)
;;   status.set(text value) raw.extra(json)
;; Every control argument is optional; a second call keeps what the first
;; set. Absent facts are your language's none. Names are snake_case here;
;; your language cases them its own way. Angles are degrees from the +x
;; axis; pitch is positive up.
;; Manual: https://clunkyard.com/manual#sdk
(import '[clunkyard Robot Clunkyard Drive Shape])

(defn steer [angles towards current]
  (.clamp ^clunkyard.Angles angles (/ (.angleDiff ^clunkyard.Angles angles (double towards) (double current)) 45)))




(defn turn [^Robot bot]
  (let [self (.me bot), position (.position self), angles (.angles bot), gun (.gun bot), scanner (.scanner bot), drive (.drive bot), status (.status bot)
        target (.orElse (.nearest (.sensors bot)) nil), cue (.orElse (.cue (.tracker bot)) nil)
        free (and (.is3d (.match bot)) (zero? (.z (.gravity (.arena bot)))))]
    (.forces drive (Drive/brake 0.2))
    (cond
      target
      (let [throttle (cond (> (.range ^clunkyard.Contact target) 350) 0.7 (< (.range ^clunkyard.Contact target) 200) -0.4 :else 0.2)
            power (.powerForFlightTime gun (.range ^clunkyard.Contact target) 1.0)
            aim (.orElse (.lead gun target power) nil)
            heading (if aim (.heading ^clunkyard.Solution aim) (.bearing ^clunkyard.Contact target))
            pitch (if aim (.pitch ^clunkyard.Solution aim) (.elevation ^clunkyard.Contact target))
            now (.aimNow gun)]
        (.set drive throttle (steer angles (.bearing ^clunkyard.Contact target) (.heading self)) (steer angles (.elevation ^clunkyard.Contact target) (.pitch self)))
        (.set scanner (.bearing ^clunkyard.Contact target) 20.0 (.elevation ^clunkyard.Contact target))
        (.aim gun heading pitch)
        (when (and (< (Math/abs (.angleDiff angles heading (.heading now))) 3) (< (Math/abs (- pitch (.pitch now))) 3) (.canFire gun power 10.0)) (.fire gun power))
        (.set status "TRACK" (.range ^clunkyard.Contact target)))
      cue
      (let [^clunkyard.Cue cue cue, bearing (.bearingTo angles position (.position cue)), distance (.distance angles position (.position cue))
            elevation (if free (.elevationTo angles position (.position cue)) 0.0), error (min (.bearingError cue) 30.0), tick (.tick (.clock bot))
            across (* (/ (- (mod tick 5) 2) 2.0) error), up (if free (* (/ (- (mod (quot tick 5) 3) 1) 2.0) error) 0.0)]
        (.set drive 0.7 (steer angles bearing (.heading self)) (steer angles elevation (.pitch self)))
        (.set scanner (+ bearing across) (.arcForReach scanner (+ distance 100)) (+ elevation up))
        (.set status "CUE" distance))
      :else
      (do (.set drive 0.5 0.33 (steer angles 0.0 (.pitch self)))
          (.set scanner (.normalize angles (+ (.heading (.last scanner)) 36)) 40.0 (if free (* (- (mod (quot (.tick (.clock bot)) 10) 3) 1) 35.0) 0.0))
          (.set status "SEARCH")))
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
