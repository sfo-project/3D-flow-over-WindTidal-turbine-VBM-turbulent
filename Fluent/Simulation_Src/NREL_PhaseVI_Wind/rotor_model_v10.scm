; Define a function in order to create rpvars for variables if they don't exist

(define (make-new-rpvar name default type)
  (if (not (rp-var-object name))
    (rp-var-define name default type #f)))

; rpvars for rotor model

(make-new-rpvar 'number-rotor-zones-var 1 'integer)
(make-new-rpvar 'rotor '() 'list)
(make-new-rpvar 'surfaces '() 'thread-list)
;(make-new-rpvar 'wall-surf '() 'thread-list)
(make-new-rpvar 'list-length-check 0 'integer)


(define gui-rotor
	(let ((panel #f)
	      (box)
	      (box1)
	      (box2)
	      (box3)
	      (box4)
	      (box5)
	      (box6)
	      (box7)
	      (box8)
	      (box9)
	      (box10)
	      (box11)
	      (box12)
	      (box13)
	      (box14)
	      (box15)
	      (number-rotor-zones)
       	      (active-rotor-zone)
	      (set)
	      (number-of-blades)
	      (rotor-radius)
	      (rotor-speed)
	      (tip-effect)
	      (rotor-origin-x)(rotor-origin-y)(rotor-origin-z)
	      (rotor-disk-pitch-angle)(rotor-disk-bank-angle)
	      (blade-pitch-collective)(blade-pitch-cyclic-sin)(blade-pitch-cyclic-cos)
	      (blade-flap-cone)(blade-flap-cyclic-sin)(blade-flap-cyclic-cos)
	      (surfaces)
	      (trim) 			; is for toggle button
	      (trimm) 			; is for tab button
	      (collective-pitch)        ; is for toggle button
	      (cyclic-pitch)        	; is for toggle button
	      (damping-factor)
	      (update-frequency)
	      (desired-thrust-coeff)(desired-x-mom-coeff)(desired-y-mom-coeff)
	      (wall-surf)
	      (geomet) 			; is for tab button
	      (number-of-sections)
	      (table-lines 1)
	      (number-list '())
	      (radius-list '())
	      (chord-list '())
	      (twist-list '())
	      (source-file-list '())
	      (number-value '())
	      (radius-value '())
	      (chord-value '())
              (twist-value '())
	      (source-file-value '())
	      (row-lines 0)
	      (r0c0)
	      (r0c1)
	      (r0c2)
	      (r0c3)
	      (r0c4)
	      (trim-check 1)
	      )

; this fn. is called every time the panel is opened using 
; define->models->rotor model-> Rotor Inputs

	(define (update-cb . args)
		(cx-set-integer-entry number-rotor-zones (rpgetvar 'number-rotor-zones-var))
		(cx-set-integer-entry active-rotor-zone 1)
		(setactive-cb) ; required when the user opens the saved case file
		(old-cb) ; is required when the user opens the panel w/o closing the session or w/o writing the case

		)

;this fn.- apply-cb  will be called when the user clicks on "close" button

	(define (apply-cb . args)
		(rpsetvar 'number-rotor-zones-var (cx-show-integer-entry number-rotor-zones ))
		(if (pair? (rpgetvar 'rotor))
			(rpsetvar 'list-length-check 1)
			(rpsetvar 'list-length-check 0) )

; This portion of the code is added if the user reduces the number of rotor-zones - say from 4 to 2 - from here
		(if (< (rpgetvar 'number-rotor-zones-var) (length(rpgetvar 'rotor)) )
		  (begin
		  (let ( (diff) (rotor (rpgetvar 'rotor)) )
		  	(set! diff (- (length rotor) (rpgetvar 'number-rotor-zones-var) ) ) 
			(set! rotor (reverse (list-tail (reverse rotor) diff) ) )
		  (rpsetvar 'rotor rotor)
		  )
		  )
		)
; Up to here
			
		(%udf-on-demand 'rotor_inputs::libudf) ; when the user clicks on OK button, the UDF will be executed 
	)

; this fn. - set-cb will be called when the user clicks on "Set..." button
; user should click on set button when he inputs some values for the 
; "active" rotor zone

; "trimming" and "geometry" are local lists inside set-cb function

	(define (set-cb . args)
		(let ((num) (col-p) (cyc-p) (dtc) (dxmc) (dymc) (general) (trimming) (geometry) (rotor (rpgetvar 'rotor)) )
		 (set! num (cx-show-integer-entry active-rotor-zone) )
		 (set! general (list (cx-show-integer-entry number-of-blades )
		 		     (cx-show-real-entry rotor-radius)
		 		     (cx-show-real-entry rotor-speed)
				     (cx-show-real-entry tip-effect) 
				     (cx-show-real-entry rotor-origin-x)
				     (cx-show-real-entry rotor-origin-y)
				     (cx-show-real-entry rotor-origin-z)
				     (cx-show-real-entry rotor-disk-pitch-angle)
				     (cx-show-real-entry rotor-disk-bank-angle)
				     (cx-show-real-entry blade-pitch-collective)
				     (cx-show-real-entry blade-pitch-cyclic-sin)
				     (cx-show-real-entry blade-pitch-cyclic-cos)
				     (cx-show-real-entry blade-flap-cone)
				     (cx-show-real-entry blade-flap-cyclic-sin)
				     (cx-show-real-entry blade-flap-cyclic-cos)
				) )
	         (rpsetvar 'surfaces
			(map thread-name->id
                     	(cx-show-symbol-list-selections surfaces)))
		 (if (= (length(rpgetvar 'surfaces)) 0)
			(begin
			(format "\n\n Error: Please select a surface for Rotor Face Zone\n") 
			(rpsetvar 'surfaces (list 0) )
			)
		 )

		 (set! general (list-add general (rpgetvar 'surfaces) ) )

		 (if (cx-show-toggle-button trim)
		  (begin
		   (if (cx-show-toggle-button collective-pitch)
			(begin
			(set! col-p 1) 
			(set! dtc (cx-show-real-entry desired-thrust-coeff) )
			)
			(set! col-p 0) 
		   )

		   (if (cx-show-toggle-button cyclic-pitch)
			(begin
			(set! cyc-p 1) 
			(set! dxmc (cx-show-real-entry desired-x-mom-coeff) )
			(set! dymc (cx-show-real-entry desired-y-mom-coeff) )
			)
			(set! cyc-p 0) 
		   )

		    (set! trimming (list trim-check 
				       col-p 
				       cyc-p 
		 		      (cx-show-integer-entry update-frequency)
				      (cx-show-real-entry damping-factor)
		         	      ))

		   (if (cx-show-toggle-button collective-pitch)
			(set! trimming (list-add trimming dtc) )
		   )

		   (if (cx-show-toggle-button cyclic-pitch)
			(begin
			(set! trimming (list-add trimming dxmc) )
			(set! trimming (list-add trimming dymc) )
			)
		   )

		  )

		  (begin
		  (set! trimming (list 0) ) 
		  (format "\n Warning: Trimming is off") 
		  )

		 )

		 (set! geometry (list (cx-show-integer-entry number-of-sections)) )
		 (set! number-value '())
		 (set! radius-value '())
		 (set! chord-value '())
		 (set! twist-value '())
		 (set! source-file-value '())
			 (let loop ((i 0))
            		  (set! number-value (list-add number-value 
 					 (cx-show-text-entry (list-ref number-list i)) ))
			  (set! radius-value (list-add radius-value 
				         (cx-show-real-entry (list-ref radius-list i)) ))
			  (set! chord-value (list-add chord-value
					 (cx-show-real-entry (list-ref chord-list i)) ))
			  (set! twist-value (list-add twist-value
					 (cx-show-real-entry (list-ref twist-list i)) ))
			  (set! source-file-value (list-add source-file-value
					 (cx-show-text-entry (list-ref source-file-list i)) ))

			  (set! i (+ i 1))
			  (if (< i (cx-show-integer-entry number-of-sections))
					 (loop i)) )

	  	(set! geometry (list-add geometry number-value )) 
	  	(set! geometry (list-add geometry radius-value )) 
	  	(set! geometry (list-add geometry chord-value )) 
	  	(set! geometry (list-add geometry twist-value )) 
		(set! geometry (list-add geometry source-file-value ))

		(if (assoc num rotor)
		 (begin
		  (let (( r (list general geometry trimming )))
			(set-cdr! (assoc num rotor) r) ) )
		(set! rotor (list-add rotor (list num general geometry trimming ) ) ) ) 
		(rpsetvar 'rotor rotor) )
		) 

; this fn. - old-cb is called when the active rotor zone "value" is changed!
; it overwrites the values kept in that particular rotor zone when the active rotor zone is
; changed! For that particular zone, if the user didn't press change/create, 
; then default values will be written 

	(define (old-cb . args)
	 (let ( (num) (dummy-rotor (rpgetvar 'rotor)) )
	  (set! num (cx-show-integer-entry active-rotor-zone))
	 (if (assoc num dummy-rotor)
		(let ( (gen) (geo) (tri) (temp) )
		 (set! temp (assoc num dummy-rotor))
		 (set! gen (list-ref temp 1))
		 (set! geo (list-ref temp 2))
		 (set! tri (list-ref temp 3))

; General Update

		 (cx-set-integer-entry number-of-blades (list-ref gen 0))
		 (cx-set-real-entry rotor-radius (list-ref gen 1))
		 (cx-set-real-entry rotor-speed (list-ref gen 2))
		 (cx-set-real-entry tip-effect (list-ref gen 3))
		 (cx-set-real-entry rotor-origin-x (list-ref gen 4))
		 (cx-set-real-entry rotor-origin-y (list-ref gen 5))
		 (cx-set-real-entry rotor-origin-z (list-ref gen 6))
		 (cx-set-real-entry rotor-disk-pitch-angle (list-ref gen 7))
		 (cx-set-real-entry rotor-disk-bank-angle (list-ref gen 8))
		 (cx-set-real-entry blade-pitch-collective (list-ref gen 9))
		 (cx-set-real-entry blade-pitch-cyclic-sin (list-ref gen 10))
		 (cx-set-real-entry blade-pitch-cyclic-cos (list-ref gen 11))
		 (cx-set-real-entry blade-flap-cone (list-ref gen 12))
		 (cx-set-real-entry blade-flap-cyclic-sin (list-ref gen 13))
		 (cx-set-real-entry blade-flap-cyclic-cos (list-ref gen 14))
		 (surfaceupdate-cb) ; require if the user opens the panel for the 2nd time
		 ; also if the user creates any extra interior surface b/w 1st & 2nd opening of the panel

		 (if (not (= (list-ref (list-ref gen 15) 0) 0) )
		 (cx-set-symbol-list-selections surfaces (map thread-id->name (list-ref gen 15) ))
		 )

; Geometry Update 

		 (cx-set-integer-entry number-of-sections (list-ref geo 0))
		 (number-cb)

		 (if (not (null? (list-ref geo 1)) )
		  (if (not (= (length number-list) (list-ref geo 0) ) )
		   (begin
			(let loop ((i 0))
                          (cx-set-text-entry (list-ref number-list i) (list-ref (list-ref geo 1) i))
                          (cx-show-item (list-ref number-list i) )
                          (cx-set-real-entry (list-ref radius-list i) (list-ref (list-ref geo 2) i))
                          (cx-show-item (list-ref radius-list i) )
                          (cx-set-real-entry (list-ref chord-list i) (list-ref (list-ref geo 3)  i))
                          (cx-show-item (list-ref chord-list i) )
                          (cx-set-real-entry (list-ref twist-list i) (list-ref (list-ref geo 4)  i))
                          (cx-show-item (list-ref twist-list i) )
                          (cx-set-text-entry (list-ref source-file-list i) (list-ref (list-ref geo 5) i))
                          (cx-show-item (list-ref source-file-list i) )
                          (set! i (+ i 1))
                          (if (< i (list-ref geo 0) )
                                (loop i) ) )

		   )
		 )
		)

		 (if (not (null? (list-ref geo 1)) )
		  (begin
		 	(let loop ((i 0))
			  (cx-set-text-entry (list-ref number-list i) (list-ref (list-ref geo 1) i))
			  (cx-show-item (list-ref number-list i) )
			  (cx-set-real-entry (list-ref radius-list i) (list-ref (list-ref geo 2) i))
			  (cx-show-item (list-ref radius-list i) )
			  (cx-set-real-entry (list-ref chord-list i) (list-ref (list-ref geo 3)  i))
			  (cx-show-item (list-ref chord-list i) )
			  (cx-set-real-entry (list-ref twist-list i) (list-ref (list-ref geo 4)  i))
			  (cx-show-item (list-ref twist-list i) )
			  (cx-set-text-entry (list-ref source-file-list i) (list-ref (list-ref geo 5) i))
			  (cx-show-item (list-ref source-file-list i) )
			  (set! i (+ i 1))
			  (if (< i (list-ref geo 0) )
				(loop i) ) ) 
			)
		 )

; Trim Update

		 (if (= (list-ref tri 0) 1) 
		   (begin
		     (cx-set-toggle-button trim #t)
		     (cx-enable-tab-button trimm #t)
		     (cx-set-integer-entry update-frequency (list-ref tri 3))
		     (cx-set-real-entry damping-factor (list-ref tri 4))
		     	(if (= (list-ref tri 1) 1) 
				(begin
				(cx-set-toggle-button collective-pitch #t)
				(cx-enable-item desired-thrust-coeff #t)
		     		(cx-set-real-entry desired-thrust-coeff (list-ref tri 5))
				)
				(begin
			 	(cx-set-toggle-button collective-pitch #f)
				(cx-enable-item desired-thrust-coeff #f)
				)
		    	)

		     	(if (= (list-ref tri 2) 1) 
				(begin
				  (cx-set-toggle-button cyclic-pitch #t) 
		    		  (cx-enable-item desired-x-mom-coeff #t)
		    		  (cx-enable-item desired-y-mom-coeff #t)

				  (if (= (list-ref tri 1) 1)
			  	    (begin
		     		    (cx-set-real-entry desired-x-mom-coeff (list-ref tri 6))
		     		    (cx-set-real-entry desired-y-mom-coeff (list-ref tri 7))
				    )
				    (begin
				    (cx-set-real-entry desired-x-mom-coeff (list-ref tri 5))
				    (cx-set-real-entry desired-y-mom-coeff (list-ref tri 6))
				    )
				  )
				)
		    		(begin
		    		(cx-set-toggle-button cyclic-pitch #f)
		    		(cx-enable-item desired-x-mom-coeff #f)
		    		(cx-enable-item desired-y-mom-coeff #f)
		    		)
		    	)
		    )
		   (begin
		   (cx-set-toggle-button trim #f) 
		   (cx-enable-tab-button trimm #f)
		   ) 
		 )

		)

; if the active rotor zone is visited for the first time

		(begin
		 (cx-set-integer-entry number-of-blades 1)
		 (cx-set-real-entry rotor-radius 0)
		 (cx-set-real-entry rotor-speed 0)
		 (cx-set-real-entry tip-effect 96)
		 (cx-set-real-entry rotor-origin-x 0)
		 (cx-set-real-entry rotor-origin-y 0)
		 (cx-set-real-entry rotor-origin-z 0)
		 (cx-set-real-entry rotor-disk-pitch-angle 0)
		 (cx-set-real-entry rotor-disk-bank-angle 0)
		 (cx-set-real-entry blade-pitch-collective 0)
		 (cx-set-real-entry blade-pitch-cyclic-sin 0)
		 (cx-set-real-entry blade-pitch-cyclic-cos 0)
		 (cx-set-real-entry blade-flap-cone 0)
		 (cx-set-real-entry blade-flap-cyclic-sin 0)
		 (cx-set-real-entry blade-flap-cyclic-cos 0)
		 (surfaceupdate-cb)
	
; if the user goes from one active rotor zone to other - start

		(if (> (cx-show-integer-entry active-rotor-zone) 1)
		  (if (>= (cx-show-integer-entry number-of-sections) 1)
		   (begin
		    (if (= (cx-show-integer-entry number-of-sections) 1)
		     (begin
 	         	(cx-set-text-entry (list-ref number-list 0) "1")
		 	(cx-set-real-entry (list-ref radius-list 0) 0)
		 	(cx-set-real-entry (list-ref chord-list 0) 0)
		 	(cx-set-real-entry (list-ref twist-list 0) 0)
		 	(cx-set-text-entry (list-ref source-file-list 0) "")
		     )
		   )
		     (if (> (cx-show-integer-entry number-of-sections) 1)
			(begin
			    (let loop ( (i table-lines) )
			        (set! i (- i 1) )
				(cx-hide-item (list-ref number-list i) )
				(cx-hide-item (list-ref radius-list i) )
				(cx-hide-item (list-ref chord-list i))
				(cx-hide-item (list-ref twist-list i))
				(cx-hide-item (list-ref source-file-list i))
				(if (> i 1)
				  (loop i) )
			    )
			    (let loop ( (i table-lines) )
				(set! i (- i 1) )
		 	    	(cx-set-real-entry (list-ref radius-list i) 0)
		 	    	(cx-set-real-entry (list-ref chord-list i) 0)
		 	    	(cx-set-real-entry (list-ref twist-list i) 0)
		 	    	(cx-set-text-entry (list-ref source-file-list i) "")
				(if (> i 0)
				  (loop i) )
			   )
			 )
		      )
		 	  (cx-set-integer-entry number-of-sections 1)
			  (set! table-lines (cx-show-integer-entry number-of-sections))
		   )
		  )
	        )
		
; end

		 (cx-set-toggle-button collective-pitch #f)
		 (cx-set-toggle-button cyclic-pitch #f)
		 (cx-set-real-entry damping-factor 0.1)
		 (cx-set-integer-entry update-frequency 10)
		 (cx-set-real-entry desired-thrust-coeff 0)
		 (cx-enable-item desired-thrust-coeff #f)
		 (cx-set-real-entry desired-x-mom-coeff 0)
		 (cx-enable-item desired-x-mom-coeff #f)
		 (cx-set-real-entry desired-y-mom-coeff 0)
		 (cx-enable-item desired-y-mom-coeff #f)
		 (cx-set-toggle-button trim #f)
		 (cx-enable-tab-button trimm #f)
	        ) ) ) )

; this fn. - setactive-cb makes sure that the value of active rotor zone can't be incremented
; without changing the value of number of rotor zones

	(define (setactive-cb . args)
	  (let ( (temp) )
	    (set! temp (cx-show-integer-entry number-rotor-zones))
	    (cx-set-attributes active-rotor-zone 'maximum temp) 
	    	(if (< temp (cx-show-integer-entry active-rotor-zone) )
	    	(cx-set-integer-entry active-rotor-zone 1) 
	    	)
	  ) 

	)

; this fn. toggle-cb switches on the trimming "tab" as soon as 
; the trimming "toggle" button is switched "on"

	(define (toggle-cb w label . args)
	  (if (string=? label "Trimming")
	  (cx-enable-tab-button trimm (cx-show-toggle-button trim)))
	  )

	
; this fn. desired-col-cb activates desired thrust coeff once the
; collective pitch toggle button is switched "on"

	(define (desired-col-cb w label . args)
	 (if (string=? label "Collective pitch")
		(cx-enable-item desired-thrust-coeff (cx-show-toggle-button collective-pitch))
	 )
	)

; this fn. desired-cyc-cb activates desired x & y mom coeffs once the
; cyclic pitch toggle button is switched "on"

	(define (desired-cyc-cb w label . args)
	 (if (string=? label "Cyclic pitch")
		(begin
		(cx-enable-item desired-x-mom-coeff (cx-show-toggle-button cyclic-pitch))
		(cx-enable-item desired-y-mom-coeff (cx-show-toggle-button cyclic-pitch))
		)
	 )
	)

; gets only interior surfaces under rotor face zone
	(define (surfaceupdate-cb . args)
		(cx-set-list-items surfaces
                         (map thread-name
                              (get-threads-of-type 'interior)))
	)

; gets only wall surfaces under consider faces for trimming
;	(define (wallsurfaces-cb . args)
;		(cx-set-list-items wall-surf
;			(map thread-name
;				(get-threads-of-type 'wall)))
;	)


;--------------------------------------------
; GEOMETRY - Hub - Number of Sections Details  
;--------------------------------------------

	(define (number-cb)
		(let ((n (cx-show-integer-entry number-of-sections)))
			(if (> n table-lines)
			  (do ((i (- n table-lines) (- i 1)))
				  ((zero? i))
				  (add-table-line) )
			)
			(if (< n table-lines)
				(delete-table-line) )
		)
	)

	(define new-hub-name
		(lambda (n)
			(set! n (+ 1 n) )
			(format #f "~a" n) ) )

	(define (add-table-line)
	 (cx-set-redisplay #f)
		(if (<= (length number-list) table-lines)
			(begin
			 (set! r0c0 (cx-create-text-entry box13 "" 'row table-lines 'col 0 'width 3 
								'value (new-hub-name (- table-lines 0))
								'editable #f ))
			 (set! number-list (list-add number-list r0c0))

			 (set! r0c1 (cx-create-real-entry box13 "" 'row table-lines 'col 1 'width 3
								'value 0.0 'minimum 0.0 'maximum 1.0) )
			 (set! radius-list (list-add radius-list r0c1))

			 (set! r0c2 (cx-create-real-entry box13 "" 'row table-lines 'col 2 'width 3
								'value 0.0 'units-quantity 'length) )
			 (set! chord-list (list-add chord-list r0c2))

			 (set! r0c3 (cx-create-real-entry box13 "" 'row table-lines 'col 3 'width 3
								'value 0.0 'units-quantity 'angle) )
			 (set! twist-list (list-add twist-list r0c3))

			 (set! r0c4 (cx-create-text-entry box13 "" 'row table-lines 'col 4 'width 9) )
			 (set! source-file-list (list-add source-file-list r0c4) )
								
			 	
			)

			(begin
			 (cx-show-item (list-ref number-list table-lines) #t)
			 (cx-show-item (list-ref radius-list table-lines) #t)
			 (cx-show-item (list-ref chord-list table-lines) #t)
			 (cx-show-item (list-ref twist-list table-lines) #t) 
			 (cx-show-item (list-ref source-file-list table-lines) #t)
			) 
		)

		         
	 (set! table-lines (1+ table-lines) )
	 (cx-set-redisplay #t) )

	(define (delete-table-line)
	 (set! row-lines (cx-show-integer-entry number-of-sections))
	 (cx-set-redisplay #f)
		
		(let loop ( (i table-lines) )
			(set! i (- i 1))
			(cx-hide-item (list-ref number-list i))
			(cx-hide-item (list-ref radius-list i))
			(cx-hide-item (list-ref chord-list i))
			(cx-hide-item (list-ref twist-list i))
			(cx-hide-item (list-ref source-file-list i))
			(if (> i row-lines)
				(loop i) ) )

		(set! table-lines (cx-show-integer-entry number-of-sections))
		(cx-set-redisplay #t)
	)

;			----------
;			ROTOR  GUI
;			----------

	(lambda args

	 (if (not panel)
	  (let ( (tabel) (frame) (frame1) (frame2) )

		(set! panel (cx-create-panel "Rotor Inputs" 'apply-callback apply-cb 
					     'update-callback update-cb
					     ) )
		(set! box (cx-create-frame panel #f 'right-of 0 'below 0 ) )
		(set! number-rotor-zones (cx-create-integer-entry box "Number of Rotor Zones" 'width 3
								'minimum 1 'maximum 10 'label-position 'left-of
								'right-of 0 'below 0 'value 1
								'activate-callback setactive-cb) )

		(set! box4 (cx-create-table panel #f 'right-of 0 'below box) )

		(set! active-rotor-zone (cx-create-integer-entry box4 "Active Rotor Zone" 'width 3
								'minimum 1 'label-position 'left-of
								'row 0 'col 0
								'activate-callback old-cb ) )
		(set! set (cx-create-button box4 "Change/Create" 'row 0 'col 1 
							'activate-callback set-cb ) )
		

		(set! box15 (cx-create-table panel #f 'right-of 0 'below box4) )
		(set! trim (cx-create-toggle-button box15 "Trimming"
							'activate-callback toggle-cb 
							'state #f 'row 0 'col 0) )
		(cx-set-toggle-button trim #f)

		(set! box1 (cx-create-frame panel #f 'right-of 0 'below box15 'tabbed #t) )

;------------
; General TAB
;------------


		(set! frame (cx-create-frame box1 "General" 'right-of 0 'below 0 ) )

		(set! frame1 (cx-create-frame frame "" 'right-of 0 'below 0) )
		(set! number-of-blades (cx-create-integer-entry frame1 "Number of Blades" 'width 3
								'minimum 1 'label-position 'left-of
								'value 1 'below 0 'left-of 0) )
		(set! rotor-radius (cx-create-real-entry frame1 "Rotor Radius" 'width 5
							 'minimum 0.0 'label-position 'left-of
							 'value 0.0 'below number-of-blades
							  'units-quantity 'length 'left-of 0))
		(set! rotor-speed (cx-create-real-entry frame1 "Rotor Speed" 'width 5
							'label-position 'left-of
							'value 0.0 'below rotor-radius
								'units-quantity 'angular-velocity 'left-of 0) )
		(set! tip-effect (cx-create-real-entry frame1 "Tip Effect (%)" 'width 5
						       'minimum 0.0 'label-position 'left-of
						       'value 96.0 'below rotor-speed 'left-of 0) )

		(set! box2 (cx-create-table frame1 "Rotor Disk Origin" 'right-of 0
								'table-options '("right-justify" "top-justify")
								'below tip-effect))
		(set! rotor-origin-x (cx-create-real-entry box2 "X" 'units-quantity 'length 'width 5
							'label-position 'left-of
							'value 0.0 'row 0 'col 0) )
		(set! rotor-origin-y (cx-create-real-entry box2 "Y" 'units-quantity 'length 'width 5
							'label-position 'left-of
							'value 0.0 'row 1 'col 0) )
		(set! rotor-origin-z (cx-create-real-entry box2 "Z" 'units-quantity 'length 'width 5
							'label-position 'left-of
							'value 0.0 'row 2 'col 0) )

		(set! box3 (cx-create-table frame1 "Rotor Face Zone" 'right-of box2 'below tip-effect
							'table-options '("right-justify" "top-justify")) )
		(set! surfaces (cx-create-symbol-list box3 "Surfaces"
						                   'visible-lines 4 'width 12 
						                   'multiple-selections #f
                   							'row 1 'col 0))
		(cx-set-list-items surfaces
                               (map thread-name
				    (get-threads-of-type 'interior)))

		(set! frame2 (cx-create-frame frame "" 'right-of frame1 'below 0) )

		(set! box5 (cx-create-table frame2 "Rotor Disk" 'below 0 'table-options
									 '("right-justify" "top-justify") ) )
		(set! rotor-disk-pitch-angle (cx-create-real-entry box5 "Pitch Angle" 'units-quantity 'angle
									'row 0 'col 0 'width 5
									'label-position 'left-of ) )									
		(set! rotor-disk-bank-angle (cx-create-real-entry box5 "Bank Angle" 'units-quantity 'angle
									'row 1 'col 0 'width 5
									 'label-position 'left-of) )


		(set! box6 (cx-create-table frame2 "Blade Pitch" 'below box5 'table-options
								'("right-justify" "top-justify") ) )
		(set! blade-pitch-collective (cx-create-real-entry box6 "Collective" 'units-quantity 'angle
									'row 0 'col 0 'width 5
									'label-position 'left-of ) )
		(set! blade-pitch-cyclic-sin (cx-create-real-entry box6 "Cyclic Sin" 'units-quantity 'angle
									'row 1 'col 0 'width 5
									'label-position 'left-of ) )
		(set! blade-pitch-cyclic-cos (cx-create-real-entry box6 "Cyclic Cos" 'units-quantity 'angle
									'row 2 'col 0 'width 5
									'label-position 'left-of ) )									
		(set! box7 (cx-create-table frame2 "Blade Flapping" 'below box6 'table-options
								'("right-justify" "top-justify") ) )
		(set! blade-flap-cone (cx-create-real-entry box7 "Cone" 'units-quantity 'angle
									'row 0 'col 0 'width 5
									'label-position 'left-of ) )
		(set! blade-flap-cyclic-sin (cx-create-real-entry box7 "Cyclic Sin" 'units-quantity 'angle
									'row 1 'col 0 'width 5
									'label-position 'left-of ) )
		(set! blade-flap-cyclic-cos (cx-create-real-entry box7 "Cyclic Cos" 'units-quantity 'angle
									'row 2 'col 0 'width 5
									'label-position 'left-of ) )

;-------------
; Geometry TAB
;-------------


		(set! geomet (cx-create-frame box1 "Geometry" 'right-of 0 'below 0  ) )

		(set! box12 (cx-create-table geomet "" 'right-of 0 'below 0) )

		(set! number-of-sections (cx-create-integer-entry box12 "Number of Sections"
						'minimum 1 'label-position 'left-of
						'value 1 'row 0 'col 0 'maximum 20 'width 3
						'activate-callback number-cb))
		
		(set! box13 (cx-create-table geomet "Hub" 'right-of 0 'below box12
								'visible-rows 5) ) 

		(set! number-list (list-add number-list (cx-create-text-entry box13 "No."
								'row 0 'col 0
							'width 3 'value "1"
							'editable #f) ) )
		(set! radius-list (list-add radius-list (cx-create-real-entry box13 "Radius (r/R)"
								'value 0 'row 0 'col 1
								'width 3 'minimum 0.0 'maximum 1.0
								) ) )
		(set! chord-list (list-add chord-list (cx-create-real-entry box13 "Chord"
								'value 0 'row 0 'col 2
								'width 3 'minimum 0.0
								'units-quantity 'length) ) )
		(set! twist-list (list-add twist-list (cx-create-real-entry box13 "twist"
								'value 0 'row 0 'col 3
								'width 3
								'units-quantity 'angle) ) )

		(set! source-file-list (list-add source-file-list (cx-create-text-entry box13 
							"File Name" 'width 9
							'row 0 'col 4 ) ) )

		(set! box14 (cx-create-table geomet "Tip" 'right-of 0 'below box13 'border #f
								'visible-rows 0) ) 

;-------------
; Trimming TAB 
;-------------


		(set! trimm (cx-create-frame box1 "Trimming" 'right-of 0 'below 0 ) )
		(set! box8 (cx-create-table trimm "" 'below 0 'right-of 0
							'table-options '("right-justify" "top-justify") ))
		(set! collective-pitch (cx-create-toggle-button box8 "Collective pitch"
							'activate-callback desired-col-cb
							'state #f 'row 0 'col 0) )
		(cx-set-toggle-button collective-pitch #f)
		(set! cyclic-pitch (cx-create-toggle-button box8 "Cyclic pitch"
							'activate-callback desired-cyc-cb
							'state #f 'row 0 'col 1) )
		(cx-set-toggle-button cyclic-pitch #f)


		(set! box9 (cx-create-table trimm "" 'below box8 'right-of 0 
						'table-options '("right-justify" "top-justify"))) 
		(set! update-frequency (cx-create-integer-entry box9 "Update Frequency" 'width 3
								'minimum 1 'label-position 'left-of
								'value 10 'row 0 'col 0) )
		(set! damping-factor (cx-create-real-entry box9 "Damping Factor" 'width 5
							   'minimum 0.0 'label-position 'left-of
							   'value 0.1 'row 1 'col 0))
		(set! desired-thrust-coeff (cx-create-real-entry box9 "Desired thrust coefficient" 'width 5
								'minimum 0.0 'label-position 'left-of
								'row 2 'col 0 ))
		(cx-enable-item desired-thrust-coeff #f) 

		(set! desired-x-mom-coeff (cx-create-real-entry box9 "Desired x-moment coefficient" 'width 5
								'minimum 0.0 'label-position 'left-of
								'row 3 'col 0 ))
		(cx-enable-item desired-x-mom-coeff #f) 

		(set! desired-y-mom-coeff (cx-create-real-entry box9 "Desired y-moment coefficient" 'width 5
								'minimum 0.0 'label-position 'left-of
								'row 4 'col 0 ))
		(cx-enable-item desired-y-mom-coeff #f) 

;		(set! box10 (cx-create-table trimm "Trim around Point" 'below box9 'right-of 0
;							'table-options '("right-justify" "top-justify") ) )
;		(set! trim-around-p-x (cx-create-real-entry box10 "X" 'units-quantity 'length
;									'row 0 'col 0 'width 5
;									'label-position 'left-of
;									'minimum 0.0 ) )
;		(set! trim-around-p-y (cx-create-real-entry box10 "Y" 'units-quantity 'length
;									'row 1 'col 0 'width 5
;									'label-position 'left-of
;									'minimum 0.0 ) )
;		(set! trim-around-p-z (cx-create-real-entry box10 "Z" 'units-quantity 'length
;									'row 2 'col 0 'width 5
;									'label-position 'left-of
;									'minimum 0.0 ) )
;
;		(set! box11 (cx-create-table trimm "Consider Faces for Trimming" 'right-of box10 'below box9
;							'table-options '("right-justify" "top-justify")) )
;		(set! wall-surf (cx-create-symbol-list box11 "Wall Surfaces" 'width 20
;						                   'visible-lines 4 
;						                   'multiple-selections #t
;                   							'row 1 'col 0))
;		(cx-set-list-items wall-surf 
;                               (map thread-name
;                                    (get-threads-of-type 'wall)))
									

	
	) )

	(cx-show-panel panel) 

	) ) )


(in-package models-package
    (define vbm-model (instance generic-model))
    (in-package vbm-model
        (define-method (model-name) "Virtual Blade Model")
        (define-method (get-model-setting)
                       (cond (vbm-model?
                              "On")
                             (else
                               "Off")))
        (define-method (available?)
                        (and vbm-model-available?
                             (^send available?)))
        (define-method (callback)
                       (gui-rotor)
                       (set! vbm-model? #t)
                       (cx-changed 'models)
                       )
        ))

(define vbm-model? #f)
(define vbm-model-available? #t)

(register-model-in-models-taskpage 'vbm-model)

