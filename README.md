# RoboticsProject

# Περιγραφή:

Με χρήση της πλατφόρμας ROS και συγκεκριμένης ρομποτικής διάταξης ζητείται να αναπτυχθεί εφαρμογή που θα επιτρέπει σε μία κινούμενη διάταξη να ακολουθεί τα τοιχώματα (wall following) του περιβάλλοντος στο οποίο βρίσκεται, να εκτιμά τη θέση και τον προσανατολισμό του σε γνωστό χάρτη, με χρήση επεκτεταμένου φίλτρου Kalman, καθώς και να κινείται προς επιθυμητό στόχο, αποφεύγοντας γνωστά εμπόδια.

# Περιγραφή Διάταξης:

Έπειτα από τροποποιήσεις που έγιναν η τελική διαταξη είναι η παρακάτω: 

  - Processing Unit: Raspberry Pi 3 Model 3b+
  - 9-DOF Accelerometer/Gyroscope/Magnitometer
  - 5 sonars (distance messurments)
  - 2 dc motors / 2 wheels
  - Small car aluminium base
  
Μετά από πειραματικές μετρήσεις υπολογίστηκαν τα παρακάτω στατιστικά μεγέθη:
  
  - Sonar  0.03 std
  - Controlled Speed (0.2 m/s -> 0.148 m/s, 0.0165 std)
  - Theta (0.0043 std) from magnetometer
  - Linear acceleration (0.05 std)
  - Gyro (0.001534 std)


## Calibration του μαγνητομέτρου:

Το μαγνητόμετρο δεν ήταν καλά ρυθμισμένο στην συγκεκριμένη διάταξη. Με λήψη τιμών καθώς περιστρεφόταν το μαγνητόμετρο με τον άξονα z κατακόρυφο φάνηκε ότι το ελλειψοειδές που δημιουργούσαν τα ζεύγη τιμών x,y ήταν έκκεντρο. Η συγκεκριμένη απόκλιση φάνηκε να τείνει προς την κατεύθυνση στην οποία βρισκόντουσαν οι μπαταρίες στην συγκεκριμένη διάταξη. Το λανθασμένο ελλειψοειδές και το διορθωμένο φαίνονται στο παρακάτω διάγραμμα:

![calibration](/project/scripts/magnetometer_calibration.png)

# Wall following:

Το wall following αποτελείται από τρεις καταστάσεις:

  - __State 0__

Εδώ το ρομπότ δεν βλέπει τοίχο με το left sonar. Προκειμένου να δει τοίχο και να είναι έτοιμο το ρομπότ να κινηθεί δεξιόστροφα, δίνεται γραμμική ταχύτητα 0.18 και μικρή γωνιακή ταχύτητα -0.3, ώστε να μην υπάρξει περίπτωση να μην βλέπει κανένα sonar τοίχο λόγω ασθενούς διάχυσης. Ανάλογα με το ποιο σόναρ από τα υπόλοιπα βλέπει, διακρίνεται ο αλγόριθμος σε substates, καθένα εκ των οποίων χρησιμοποιεί τον δικό του PD controller, προκειμένου να έρθει το ρομπότ όσο παράλληλα γίνεται στον τοίχο, πριν βλέπει σε επιθυμητή απόσταση το left sonar.

  - __State 1__

Εδώ το ρομπότ είναι σχεδόν παράλληλο στον τοίχο και προσπαθεί να παραμείνει παράλληλο, ακολουθώντας τον τοίχο σε επιθυμητή απόσταση.

  - __State 2__

Εδώ το ρομπότ τίθεται σε δεξιά στροφή για πειραματικά προσδιορισμένο χρονικό διάστημα. Ο αλγόριθμος φτάνει σε αυτήν την κατάσταση εάν οποιαδήποτε στιγμή δει το front sonar σε αρκετά μικρή απόσταση. Έπειτα από αυτήν την στροφή, επιστρέφει ο αλγόριθμος στο state 1.

## Πειραματικά αποτελέσματα:

Παρατίθεται ένα στιγμιότυπο από τα πειράματα που αφορούν στο πρώτο task της εργασίας.

![wallfollowing](/Videos/WallFollowing/4.gif)


# State estimation:

Για την εκτίμηση της θέσης και της γωνίας του ρομπότ χρησιμοποιήθηκε επεκτεταμένο φίλτρο Kalman. Το μοντέλο έχει διάνυσμα κατάστασης Χ την θέση και την γωνία του ρομπότ με:

<p align="center">
  <img width="330px"  src="/eq/1.gif">
</p>

 Για την εκτίμηση της θέσης χρησιμοποιήθηκαν οι μετρήσεις των σόναρ στον πίνακα μετρήσεων και στο μοντέλο μία εκτίμηση της μέσης ταχύτητας του ρομπότ όταν του δινόταν εντολή να κινηθεί με ταχύτητα 0.2. Η μέση τιμή αυτή προέκυψε 0.148 m/s από 20 μετρήσεις από τις οποίες εξήχθη και η τυπική απόκλιση 0.00474 m/s. Για την εκτίμηση της γωνίας χρησιμοποιήθηκε το μαγνητόμετρο στον πίνακα των μετρήσεων και η γωνιακή ταχύτητα στο μοντέλο. Δεδομένων αυτών, ο πίνακας Φ του φίλτρου προέκυψε:

<p align="center">
  <img  width="170px" src="/eq/2.gif">
</p>
Και ο πίνακας w:

<p align="center">
  <img width="150px"  src="/eq/3.gif">
</p>
Από τον πίνακα Φ προκύπτει ο πίνακας Α από γραμμικοποίηση που χρησιμοποιείται στο φίλτρο Kalman:
<br/><br/>
<p align="center">
  <img width="90px" src="/eq/4.gif">
</p>
Και ο πίνακας αβεβαιότητας του μοντέλου τελικά είναι:
<br/><br/>
<p align="center">
  <img width="500px" src="/eq/5.gif">
</p>
Για τον πίνακα μετρήσεων z ισχύει ότι:

<p align="center">
  <img width="150px" src="/eq/z.png">
</p>
Όπου I<sub>L</sub>, I<sub>FL</sub>, I<sub>F</sub>, I<sub>FR</sub>, I<sub>R</sub> οι μετρήσεις των σόναρ και θ<sub>imu</sub> η μέτρηση του μαγνητομέτρου.

Ο πίνακας h δημιουργείται σε κάθε βήμα ξεχωριστά, διότι μερικές μετρήσεις των σόναρ μπορεί να μην γίνουν αποδεκτές, οπότε παραλείπονται και οι αντίστοιχες γραμμές του h. Πιο συγκεκριμένα, αν η μέτρηση ενός σόναρ είναι πάνω από κάποιο όριο, πειραματικά προσδιορισμένο, η μέτρηση απορρίπτεται και μαζί του και η αντίστοιχη γραμμή στον πίνακα h. Ακόμη, μέσω του μοντέλου υπολογίζεται η εκτίμηση της θέσης και της γωνίας του ρομπότ την συγκεκριμένη χρονική στιγμή. Μέσω αυτής προσδιορίζεται ο τοίχος στον οποίο βλέπει κάθε σόναρ, καθώς και η γωνία με την οποία προσπίπτει στο τοίχωμα η ηχητική δέσμη. Αν η γωνία αυτή είναι μεγαλύτερη από ένα επίσης πειραματικά προσδιορισμένο όριο της τάξης των 25 μοιρών, τότε αυτή η μέτρηση δεν χρησιμοποιείται, καθώς από το όριο αυτό και πάνω η ανάκλαση είναι τέτοια που δεν επιτρέπει στον αισθητήρα να λάβει πίσω την διαχεόμενη ηχητική δέσμη. Για τον προσδιορισμό του τοιχώματος στο οποίο κοιτάει κάποιο σόναρ χρησιμοποιούνται οι εξισώσεις των ευθειών και τα άκρα των ευθυγράμμων τμημάτων που ορίζουν οι τοίχοι. Με βάση αυτά επίσης προσδιορίζονται και οι εξισώσεις που θα αποτελέσουν τον πίνακα h. Κάθε εξίσωση είναι της μορφής:

<p align="center">
  <img width="85px" src="/eq/7.gif">
</p>
όπου d<sub>i</sub> η απόσταση του σόναρ από τον τοίχο και θ<sub>i</sub> η κυρτή γωνία που ορίζεται από την ευθεία στην οποία βλέπει το σόναρ και την κάθετο στον τοίχο. Το συνημίτονο αυτής ισούται με το ημίτονο της κυρτής γωνίας που ορίζεται από την ευθεία του σόναρ και την ευθεία του τοίχου, πράγμα πιο εύκολα υπολογίσιμο. Όλα αυτά υπολογίζονται χρησιμοποιώντας την βιβλιοθήκη sympy της python ώστε να προκύψουν εκφράσεις με παραμέτρους τα x, y, θ του ρομπότ. Αυτές απλοποιούνται όσο γίνεται και παραγωγίζονται ως προς όλα τα στοιχεία του διανύσματος κατάστασης, ώστε να χρησιμοποιηθούν για την δημιουργία του H ύστερα, το οποίο προκύπτει από γραμμικοποίηση του h:
<br/><br/>
<p align="center">
  <img width="85px" src="/eq/8.gif">
</p>
Με αυτούς τους πίνακες έτοιμους, μπορεί να υλοποιηθεί η συνάρτηση που κάνει την αναβάθμιση του διανύσματος κατάστασης και της αβεβαιότητας με χρήση των σχέσεων που υπαγορεύει η αντίστοιχη θεωρία.
Λόγω της καθυστέρησης της python να εκτελέσει πράξεις μεταξύ πινάκων, τυγχάνει το Kalman να υπολογίσει την τρέχουσα εκτιμώμενη θέση του ρομπότ βάσει προηγούμενων ενδείξεων των σόναρς και όχι των τρέχοντων. Για να διορθωθεί αυτό, η εκτίμηση θέσης περνά στο τέλος από το μοντέλο συστήματος, με dt το χρόνο από τη στιγμή που λαμβάνουμε την μέτρηση μέχρι την στιγμή που περνάμε από το μοντέλο την εκτίμηση θέσης.

Μπορείτε να δείτε τα τέστ που κάναμε με video και τα διαγράμματα που παράχθηκαν από το estimation στον φάκελο Videos/kalman_tests. Εκεί θα βρήτε και άλλα video με τέστ που κάναμε για όλα τα task πέρα από αυτα που φαίνονται στην αναφορά (λογικά για να δείτε τα video θα πρέπει να κατεβάσετε το φάκελο του project διότι το github δεν κάνει preview τοσο μεγάλα αρχεία!).

# Αποφυγή εμποδίων με Artificial Potential Fields


### Παραδοχές:

Υπάρχει μόνο ένα, γνωστής θέσης και γεωμετρίας εμπόδιο στον χώρο κίνησης του ρομπότ. Αυτό είναι ένας κύβος 15cmΧ15cmΧ15cm και το κέντρο του βρίσκεται στην θέση (17.5cm,-17.5cm).

Το ρομπότ έχει σταθερή γραμμική ταχύτητα.

### Μοντελοποίηση του χώρου κίνησης:

Η αναπαράσταση του χώρου κίνησης αποτελείται από ένα occupancy grid με ανάλυση 10Χ10 και μέγεθος κελιών 15cmX15cm.

### Υλοποίηση της μεθόδου APF:

Κατά την έναρξη του task υπολογίζεται για κάθε κέλι στο occupancy grid η ένταση του τεχνητού potential field ακολουθώντας τις παρακάτω εξισώσεις:

<p align="center">
  <img width="240px" src="/eq/11.png">
</p>
<p align="center">
  <img width="350px" src="/eq/22.png">
</p>
<p align="center">
  <img width="480px" src="/eq/33.png">
</p>
Οι παραπάνω τιμές αποθηκεύονται σε έναν πίνακα.

Κατά την διάρκεια της εκτέλεσης του task, χρησιμοποιώντας τα δεδομένα του Kalman filter, υπολογίζεται η θέση του ρομπότ πάνω στο occupancy grid. Γνωρίζοντας το cell που βρίσκεται το ρομπότ, χρησιμοποιείται best-first αλγόριθμος για τα 8 γειτονικά κελιά, επιλέγοντας αυτό με την μικρότερη τιμή στον APF πίνακα.

Υπολογίζεται η γωνία που απαιτείται να στρίψει το ρομπότ για να πλοηγηθεί στο επιλεγμένο κελί και αυτή ακολουθείται με την χρήση ενός PD ελεκτή που καθορίζει την γωνιακή ταχύτητα.

Τέλος, όταν από τις τίμες του estimation υπολογιστεί απόσταση από τον στόχο μικρότερη των 10cm, τότε σταματάει το ρομπότ και ολοκληρώνεται το task.

Παρακάτω φαίνεται ένα παράδειγμα εκτέλεσης της παραπάνω υλοποίησης:

![potfields](/Videos/ObstacleAvoidance/PotentialFields/2.gif)


# Αποφυγή εμποδίων με Α*

Λόγω των εγγενών προβλημάτων που προκύπτουν με την μέθοδο των δυναμικών πεδίων (τοπικά ακρότατα, απότομες στροφές), έγινε και χρήση του αλγορίθμου A* για εκ των προτέρων πρόβλεψη της επιθυμητής διαδρομής και ύστερα παρακολούθησή της. Ο πλήρης αλγόριθμος είναι ο εξής:
Ο χάρτης χωρίζεται σε πλέγμα, η απόσταση των γειτονικών κόμβων του οποίου επιλέχθηκε να είναι αρκετά μικρή (περίπου 2 εκατοστά). Ο κάθε κόμβος του πλέγματος γειτονεύει και με τους 8 γύρω κόμβους. Το εμπόδιο καθιστά τους κόμβους που καταλαμβάνει, καθώς και κάποιους κόμβους ακόμα γύρω του, δεδομένης μιας απόστασης που θα θέλαμε το ρομπότ να διατηρήσει από το εμπόδιο, μη διαθέσιμους στον αλγόριθμο Α*. Ύστερα εκτελείται ο αλγόριθμος Α* για να βρεθεί μια διαδρομή από την αρχική θέση στην τελική. Δεδομένης της τοπολογίας του πλέγματος η διαδρομή αυτή θα περιλαμβάνει απότομες στροφές που ύστερα η διάταξη θα δυσκολευόταν να παρακολουθήσει. Γι' αυτόν τον λόγο ακολουθεί μία διαδικασία λείανσης της διαδρομής που προέκυψε. Αυτή εντοπίζει τα γωνιακά σημεία της διαδρομής και τα κρατάει σε έναν πίνακα. Ως γωνιακό σημείο ορίζουμε ένα σημείο για το οποίο το προηγούμενο, το ίδιο και το επόμενο στην διαδρομή δεν είναι συνευθειακά. Ύστερα για κάθε γωνιακό σημείο Γίνεται το εξής:

1. Αρχικοποιείται μία λίστα που περιέχει μόνο το γωνιακό σημείο που εξετάζεται.
2. Άν δεν υπάρχει γωνιακό σημείο σε προηγούμενο διάστημα μήκους 8 cm (πειραματικά επιλεγμένο) επιλέγεται το κοντινότερο σημείο σε όμοιο διάστημα που προηγείται στην διαδρομή. Αλλιώς επιλέγεται το προηγούμενο γωνιακό σημείο και επαναλαμβάνεται το 2.
3. Αν δεν υπάρχει γωνιακό σημείο σε ακόλουθο διάστημα μήκους 8 cm, τότε επιλέγεται το κοντινότερο σημείο σε όμοιο διάστημα που ακολουθεί στην διαδρομή. Αλλιώς επιλέγεται το επόμενο γωνιακό σημείο και επαναλαμβάνεται το 3.
3. Επιλέγεται το επόμενο γωνιακό σημείο που δεν ανήκει σε καμμία λίστα και εκτελείται το 1. Αν δεν υπάρχει, συνεχίζει στο 4.
4. Για κάθε λίστα από σημεία, αν αυτή έχει μήκος 3, δημιουργείται μια καμπύλη bezier με αυτά τα 3 σημεία. Αν έχει παραπάνω σημεία, αντί να δημιουργηθεί καμπύλη bezier ανώτερης τάξης, επιλέγονται τα midpoints όλων των σημείων ελέγχου της και δημιουργείται σύνολο διαδοχικών καμπύλων bezier δευτέρου βαθμού με άκρα τα midpoints που βρέθηκαν και το αρχικό και τελικό σημείο της καμπύλης, και σημεία ελέγχου, τα σημεία ελέγχου που προαναφέρθηκαν.
Λόγω των ιδιοτήτων των καμπυλών bezier η διαδρομή που δημιουργείται είναι παραγωγίσιμη, χωρίς γωνίες και το ρομπότ μπορεί να την ακολουθήσει πιο εύκολα. Το αποτέλεσμα του αλγορίθμου φαίνεται στο παρακάτω διάγραμμα:

![smoothing](/alpha_star/scripts/smoothing.png)

Με δεδομένο αυτό το διάγραμμα να σημειωθεί ότι αν στον Α* επιτρεπόντουσαν και διαγώνιες κινήσεις, η λείανση θα ήταν λιγότερη, καθώς θα υπήρχαν περισσότερες διαδοχικές γωνίες και θα παρατηρείτο το φαινόμενο που παρατηρείται και στο τέλος του παραπάνω διαγράμματος. Μία λύση είναι στις καμπύλες πολλών σημείων να παραληφθούν διαδοχικά σημεία που απέχουν λιγότερο από κάποιο κάτω φράγμα.

![goodsmoothing](/alpha_star/scripts/goodsmoothing.png)

Ύστερα το ρομπότ πρέπει να ακολουθήσει την καμπύλη που προέκυψε από τα παραπάνω. Γι' αυτόν τον σκοπό χρησιμοποιείται μόνο ο PD ελεγκτής που δημιουργήθηκε για τον έλεγχο της γωνίας του ρομπότ και σε αυτόν αλλάζει σε κάθε βήμα το setpoint ως εξής:

1. Λαμβάνονται τιμές x,y για την θέση της διάταξης στον χώρο από το φίλτρο Kalman.
2. Βρίσκεται το κοντινότερο σημείο p στην καμπύλη.
3. Επιλέγεται σημείο στην καμπύλη που να ακολουθεί το p με απόσταση περίπου 10cm (πειραματικά προσδιορισμένο). Αν δεν υπάρχει, επιλέγεται το τελευταίο σημείο της καμπύλης. Αυτό γίνεται δεδομένης της γνώσης του resolution του πλέγματος και των διαστάσεων του χάρτη, καθώς και του γεγονότος ότι ο αλγόριθμος που λειαίνει τις καμπύλες, προσπαθεί να κρατήσει τον ίδιο αριθμό σημείων στην ακολουθία που αποτελεί την διαδρομή.
4. Τίθεται στον PD ελεγκτή ως setpoint η γωνία που πρέπει να έχει το ρομπότ για να προσεγγίσει εκείνο το σημείο.
5. Ως είσοδος στον PD ελεγκτή δίνεται η γωνία που έχει πραγματικά το ρομπότ, όπως προκύπτει μόνο από το μαγνητόμετρο για λόγους ταχύτητας και η έξοδος του τίθεται ως γωνιακή ταχύτητα του ρομπότ.

Τα αποτελέσματα της πλήρους παραπάνω διαδικασίας φαίνονται στο παρακάτω διάγραμμα/video:

![AstarVideo](/Videos/ObstacleAvoidance/A*/1.gif)

![experiment](/alpha_star/scripts/experiment.png)

To ρομπότ ακολουθεί αρκετά καλά την διαδρομή που υπολογίστηκε και καταλήγει σε απόσταση μικρότερη των 5cm από τον τελικό στόχο. Εκεί σταμάτησε να κινείται δεδομένου του ότι το ρομπότ έχει ακτίνα πάνω από 11cm. Παρακάτω φαίνεται το διάγραμμα του σφάλματος στην παρακολούθηση της διαδρομής. Φαίνεται δηλαδή η ελάχιστη απόσταση από την καμπύλη για κάθε δείγμα.

![error](/alpha_star/scripts/error.png)

Το σφάλμα αυτό έχει μέγιστη τιμή περίπου 4.3cm και μέση τιμή λιγότερο από 1.9cm. Αυτό δείχνει ότι θα μπορούσε να επιλεχθεί και λίγο μικρότερη απόσταση τερματισμού της προσπάθειας προσέγγισης του στόχου.

# Προβλήματα/Αστοχίες Υλικού

Κατά τη διάρκεια υλοποίησης του project διαπιστώθηκαν τα εξής προβλήματα στη λειτουργία του robot:

1. Οι αισθητήρες sonar όταν βλέπουν εμπόδια/τοίχους σε γωνία μεγαλύτερη των ~30 μοιρών δεν επιστρέφουν σωστές μετρήσεις και θεωρούν ότι τα εν λόγω εμπόδια βρίσκονται πιο μακριά από ότι στην πραγματικότητα.
<br/><br/>**Αντιμετώπιση:** Φιλτράρισμα των μετρήσεων των sonars, ώστε να αγνοούνται αποπροσανατολισμένες μετρήσεις.
<br/><br/>**Πρόταση:** Η γωνία ανάμεσα στους πλάγιους αισθητήρες και στους μπροστινούς-πλάγιους να είναι 45 μοίρες αντί για 30. Έτσι θα είναι λιγότερες οι γωνίες που μπορεί να πάρει η διάταξη, στις οποίες δεν βλέπει κάποιος αισθητήρας κάποιο αντικείμενο.

2. Η μπροστινή ρόδα του ρομπότ συχνά δυσκόλευε την κίνηση του, καθώς δεν μπορούσε να στρίψει όπως έπρεπε. Κυρίως το πρόβλημα παρατηρήθηκε όταν το power bank που χρησιμοποιήθηκε βρισκόταν στη μέση του robot, στον χώρο που είχε προβλεφθεί να μείνει κενός για αυτό το σκοπό.<br/><br/>**Αντιμετώπιση:** Αλλαγή θέσης του power bank με τις μπαταρίες τροφοδοσίας των κινητήρων, ώστε να αλλάξει το κέντρο βάρους το ρομπότ και να μην έχει τόσο βάρος η μπροστινή ρόδα.
<br/><br/>**Πρόταση:** Αντικατάσταση της μπροστινής ρόδας με μια σφαιρική.

3. Κατά τη διάρκεια υλοποίησης του Kalman Filter παρατηρήθηκε ότι υπερθερμαίνεται ο επεξεργαστής του Rasberry Pi και δεν λαμβάνεται ικανοποιητικός αριθμός δειγμάτων, με αποτέλεσμα να έχουμε ίχνος της τροχιάς κίνησης του robot και όχι in real time motion.
<br/><br/>**Αντιμετώπιση:** Αλλάξαμε την πλακέτα του Rasberry Pi από τη 3 στη 3b+ (ίσως να έφταιγε η συγκεκριμένη αρχική πλακέτα που χρησιμοποιούσαμε).

4. Ο αλγόριθμος των δυναμικών πεδίων έχει εγγενή προβλήματα, αφού δημιουργεί τοπικά ελάχιστα που εξαρτώνται από την εκάστοτε τοπολογία με μη πρακτικό τρόπο αποφυγής τους εκ των προτέρων. Επίσης χρειάζονται γνώση του χώρου για τον υπολογισμό του πεδίου σε όλα τα σημεία ενδιαφέροντος.
<br/><br/>**Αντιμετώπιση:** Έγινε χρήση του αλγόριθμου Α* για τον υπολογισμό της αρχικής διαδρομής, θεωρώντας γνωστό εμπόδιο.
<br/><br/>**Πρόταση:** Λόγω του τρόπου με τον οποίο έχει υλοποιηθεί το φίλτρο Kalman, δεδομένου του ότι απορρίπτονται οι μετρήσεις των αισθητήρων απόστασης που αποκλίνουν πολύ από την αναμενόμενη τιμή τους μέσω της πρόβλεψης, μπορούν να χρησιμοποιηθούν αν είναι αρκετά μικρότερες από το αναμενόμενο για την ανίχνευση ενός έως τώρα αγνώστου εμποδίου. Σε εκείνη την στιγμή πρέπει να ξανατρέξει ο αλγόριθμος Α* για επανυπολογισμό διαδρομής που θα αποφεύγει το συγκεκριμένο εμπόδιο. Σε περίπτωση που η διαδικασία είναι αργή, προτείνεται ο αλγόριθμος D*, που έχει δημιουργηθεί ακριβώς γι' αυτόν τον σκοπό, κρατώντας τα δεδομένα των προηγούμενων προσπαθειών για εύρεση διαδρομής και ενημερώνοντάς τα κατάλληλα.
