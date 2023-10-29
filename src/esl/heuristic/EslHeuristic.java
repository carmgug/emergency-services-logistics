package esl.heuristic;

import com.sun.org.apache.xpath.internal.Arg;
import fr.uga.pddl4j.parser.TypedSymbol;
import fr.uga.pddl4j.problem.Fluent;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.Condition;
import utility.Argument;
import utility.Predicate;
import fr.uga.pddl4j.planners.statespace.search.Node;




import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class EslHeuristic {

    public final static String NAME = "EslHeuristic By Carmelo-Carmelo-Vittorio";

    private static volatile EslHeuristic instance = null;
    private final Problem problem;

    // idConditionMap: Map that translates the ID of an existing predicate within the problem
    // into a more understandable object. It allows associating an ID with a predicate,
    // providing a convenient way to access and interpret conditions within the system.
    private Map<Integer,Predicate> idConditionMap;
    // The typeToArguments map associates the argument type name with a list of instantiated arguments in the problem.
    // For example, it might look like: robot:[<-1:r1>, <-1:r2>]
    private Map<String,List<Argument>> typeToArguments;
    private List<Predicate> allPossiblePredicates;



    //
    private int n_boxes;
    private int n_carrier;
    private int n_robot;

    //Map containing pairs <String, Integer>, where String refers to the unique name of the carrier,
    // and Integer indicates the available space on that carrier.
    private Map<String,Integer> carrierInfo;



    public EslHeuristic(Problem problem){
        this.problem=problem;
        this.idConditionMap=new HashMap<>();
        this.allPossiblePredicates=new LinkedList<>();
        typeToArguments=new HashMap<>();
        createIdConditionMapAndAllPossiblePredicates();
        createTypeToArguments();


        setUpVariableOfTheProblem();

    }
    private void setUpVariableOfTheProblem(){
        //Take the number of the total boxes
        this.n_boxes=this.typeToArguments.get("box").size();
        this.n_carrier=this.typeToArguments.get("carrier").size();
        this.n_robot=this.typeToArguments.get("robot").size();

        //Determiniamo il numero di slot per ogni carrier istanziato all'interno del problema
        carrierInfo=new HashMap<>();
        for (Argument curr_carrier:this.typeToArguments.get("carrier")){
            String curr_carrier_name=curr_carrier.getArgument_name();
            Integer curr_carrier_slots=0;
            List<Argument> slots=this.typeToArguments.get("slot");
            for(Argument curr_slot:slots){
                if(curr_slot.getArgument_name().contains(curr_carrier_name)){
                    curr_carrier_slots++;
                }
            }
            carrierInfo.put(curr_carrier_name,curr_carrier_slots);
        }



    }

    private void createIdConditionMapAndAllPossiblePredicates(){
        Integer i=0;
        for (Fluent f:problem.getFluents()){
            String[] curr=problem.toString(f).replaceAll("[\\(\\)]", "").split(" ");
            Predicate curr_p=new Predicate(f.getSymbol(),curr[0],f.getArguments(),curr);
            idConditionMap.put(i,curr_p);
            allPossiblePredicates.add(curr_p);
            i++;
        }
        return ;
    }

    private void createTypeToArguments() {
        for(TypedSymbol<String> elem:problem.getParsedProblem().getObjects()){
            String type=elem.getTypes().get(0).toString();
            String obj=elem.getValue();
            //TODO ATTENZIONE NON HO MESSO UN ID ALL'ARGOMENTO PERCHè DEVO LEGGERE LA DOCUMENTAZIONE
            if(!typeToArguments.containsKey(type)) {
                typeToArguments.put(type, new LinkedList<>());
                typeToArguments.get(type).add(new Argument(-1,obj));
            }
            else
                typeToArguments.get(type).add(new Argument(-1,obj));
        }
    }

    public static EslHeuristic getInstance(Problem problem){
        if (instance == null) {
            synchronized(EslHeuristic.class) {
                if (instance == null) {
                    instance = new EslHeuristic(problem);
                }
            }
        }
        return instance;
    }
    public List<Predicate> getPredicates(int[] state){
        List<Predicate> res=new LinkedList<>();
        for (int x : state){
            res.add(this.idConditionMap.get(x));
        }
        return res;
    }


    public boolean isWorth(Node current, Action op){

        String action_name=op.getName();

        switch (action_name){
            case "move":
                return evaluateMove(current,op);
            case "move-carrier":
                return evaluateMoveCarrier(current,op);
            case "fill-box":
                return evaluateFillBox(current,op);
            case "give-content":
                return evaluateGiveContent(current,op);
            case "satisfied-with-at-least-one":
                return evaluatesatisfiedWithAtLeastOne(current,op);
            case "release-carrier":
                return false;
            case "unload-carrier":
                return false;
            default:
                return true;
        }
    }

    private boolean evaluatesatisfiedWithAtLeastOne(Node current, Action op){
        int[] parameters= op.getInstantiations(); //:parameters (satisfied-with-at-least-one ?p ?content ?content)
        int person_id=parameters[0];
        int element_id_1=parameters[1];
        int element_id_2=parameters[2];

        // Get goal not already satisfied.
        List<Predicate> state= getPredicates(current.stream().toArray()); //Predicati Veri nello stato corrente
        List<Predicate> allGoals=getPredicates(problem.getGoal().getPositiveFluents().stream().toArray());
        List<Predicate> goalAlreadySatisfied=getGoalAlreadySatisfied(state,allGoals);
        List<Predicate> goalNotAlreadySatisfied=getGoalNotAlreadySatisfied(allGoals,goalAlreadySatisfied);


        for(Predicate p:goalNotAlreadySatisfied.stream().filter(p-> p.getName().equals("satisfied-with-at-least-one")).collect(Collectors.toList())){
            //Se esiste un goal che mi esprime la necessità di dare l'element_id_1 o l'element_id_2 alla person_id allora effettuo l'azione
            if(p.containsArgByID(person_id) && p.containsArgByID(element_id_2) && p.containsArgByID(element_id_1)){
                return true;
            }
        }





        return false;

    }



    private boolean evaluateGiveContent(Node current,Action op){
        int[] parameters= op.getInstantiations(); //:parameters (?r - robot ?p - person ?elem - content ?b - box   ?l - location)
        int person_id=parameters[1];
        int element_id=parameters[2];

        // Goal not already satisfied.
        List<Predicate> state= getPredicates(current.stream().toArray()); //Predicati Veri nello stato currente
        List<Predicate> allGoals=getPredicates(problem.getGoal().getPositiveFluents().stream().toArray());
        List<Predicate> goalAlreadySatisfied=getGoalAlreadySatisfied(state,allGoals);
        List<Predicate> goalNotAlreadySatisfied=getGoalNotAlreadySatisfied(allGoals,goalAlreadySatisfied);


        for(Predicate p:goalNotAlreadySatisfied){
            if(p.containsArgByID(person_id) && p.containsArgByID(element_id)){
                return true;
            }
        }


        //Altrimenti non esiste goal che mi espre la necessità di consegnare l'element_id alla person_id.
        return false;
    }

    private boolean evaluateFillBox(Node current,Action op){
        int[] parameters= op.getInstantiations(); //:parameters (?r, ?box, ?element ,?location )
        //Id dell'elemento che voglio aggiungere
        int element_id=parameters[2];

        //Carico la box solo di cose che mi servono per raggiungere il goal
        List<Predicate> state= getPredicates(current.stream().toArray()); //Predicati Veri nello stato corrente
        List<Predicate> allGoals=getPredicates(problem.getGoal().getPositiveFluents().stream().toArray());
        List<Predicate> goalAlreadySatisfied=getGoalAlreadySatisfied(state,allGoals);
        List<Predicate> goalNotAlreadySatisfied=getGoalNotAlreadySatisfied(allGoals,goalAlreadySatisfied);

        // List of elements required to achieve the goal
        List<Argument> requiredGoalElements=getListOfElementsRequiredToAchiveTheGoal(goalNotAlreadySatisfied);
        // List of elements contained in all the boxes of the world
        List<Argument> boxContents = state.stream()
                .filter(p -> "has-inside".equals(p.getName()))
                .map(p -> p.getArguments().get(1))
                .collect(Collectors.toList());
        for(Argument arg: boxContents){
            if(requiredGoalElements.contains(arg)){
                requiredGoalElements.remove(arg);
            }
        }

        //Effettuo il check se l'elemento che voglio aggiungere è presente negli elementi richiesti.
        for(Argument arg:requiredGoalElements){
            if(arg.getArgument_id()==element_id){
                return true;
            }
        }



        return false;
    }

    private List<Argument> getListOfElementsRequiredToAchiveTheGoal(List<Predicate> goalsNotSatisfiedYet){
        return  goalsNotSatisfiedYet.stream()
                .flatMap(p -> {
                    if ("satisfied-with-at-least-one".equals(p.getName()) && p.getArguments().size() >= 3) {
                        // If the predicate is "satisfied-with-at-least-one," there are two objects
                        return Stream.of(
                                p.getArguments().get(1).clone(),
                                p.getArguments().get(2).clone()
                        );
                    } else {
                        // Otherwise, return only the second argument
                        return Stream.of(p.getArguments().get(1).clone());
                    }
                })
                .collect(Collectors.toList());
    }

    private boolean evaluateMoveCarrier(Node current,Action op){
        int[] parameters= op.getInstantiations(); //:parameters (?r, ?from, ?to ,?c )
        List<Predicate> state= getPredicates(current.stream().toArray()); //Predicati Veri nello stato currente
        List<Predicate> allGoals=getPredicates(problem.getGoal().getPositiveFluents().stream().toArray());
        List<Predicate> goalAlreadySatisfied=getGoalAlreadySatisfied(state,allGoals);
        List<Predicate> goalNotAlreadySatisfied=getGoalNotAlreadySatisfied(allGoals,goalAlreadySatisfied);
        int departure_id=parameters[1];
        int des_id=parameters[2];


        //Caso base: Se la location di partenza coincide con la location di arrivo allora sto fermo.
        if(departure_id==des_id){
            return false;
        }

        //Caso base2: Se ho già effettuato una move precedentemente siccome mi muovo verso il deposito se ho uno spazio libero o mi muovo verso
        //Destinazioni in cui ci sono persone che vogliono ciò che ho nel carrello allora non do la possibilità di rimuovermi di nuovo
        Action previous_op = current.getAction()!=-1 ? problem.getActions().get(current.getAction()) : null;
        if(previous_op!= null && previous_op.getName().equals(op.getName())){
            return false;
        }

        //Caso 1: Mi muovo verso il deposito se e soltano se ho almeno uno spazio libero sul carrello o ho una cassa vuota(Altrimenti non posso caricare nulla);
        if(state.stream().filter(x-> x.getName().equals("depot-at")).anyMatch(x -> x.getArguments().get(0).getArgument_id()==des_id)){
            //Se esiste almeno uno spazio libero sul carrello oppure se almeno una cassa su di me è vuota allora torno al deposito altrimenti no.
            int carrier_id=parameters[3];
            int empty_spaces=state.stream().filter(x-> x.getName().equals("empty")).filter(x -> x.getArguments().get(1).getArgument_id()==carrier_id).collect(Collectors.toList()).size();
            //(on-carrier ?b - box ?c - carrier)
            int empty_boxes_on_carrier=state.stream().filter(x -> x.getName().equals("on-carrier")).filter(on_carrier-> on_carrier.containsArgByID(carrier_id))
                    .map(x -> x.getArguments().get(0))//Lista di casse che sono sul carrier corrente (che stiamo muovendo)
                    .filter(box -> !state.contains(new Predicate("full",box)))//Lista di casse che sono sul carrier corrente e sono vuote;
                    .collect(Collectors.toList()).size();//(full ?b - box) //Prendiamo il numero di casse che sono sul carrier corrente e sono vuore;
            return (empty_spaces+empty_boxes_on_carrier)>0;
        }

        //Caso 3:
        //Mi muovo solo verso destinazioni, che sono diverse dal deposito, se ci sono persone che voglio ciò che ho nel carrello
        for(Predicate p: state.stream().filter(p -> p.getName().equals("at")).collect(Collectors.toList())){
            //il Predicato p ha forma (at object location)
            int location=p.getArguments().get(1).getArgument_id();
            if(des_id==location){//Analizziamo la destinazione
                int obj_id=p.getArguments().get(0).getArgument_id(); //Abbiamo l'id dell'oggetto nella posizione in cui ci vogliamo spostare
                for (Predicate curr_goal:goalNotAlreadySatisfied.stream().filter(curr_p -> curr_p.containsArgByID(obj_id)).collect(Collectors.toList())){
                    //Alla posizione des_id c'è un obj_id che vuole almeno un content;
                    //Lista degli elementi che obj_id vuole
                    List<Argument> wantedContent=curr_goal.getArguments(); //Dobbiamo rimuovere il primo elemento che si riferisce alla persona che vuole il contenuto
                    wantedContent.remove(0);
                    //Devo verificare se il content è presente nel mio carrello
                    int carrier_id=parameters[3];
                    //Prendo le casse sul carrier_id e le controllo
                    for(Argument box:typeToArguments.get("box")) {
                        //Controlla che il box è sul carrier  on-carrier ?b - box ?c - carrier
                        if (state.stream().filter(x -> x.getName().equals("on-carrier"))
                                .anyMatch(x -> x.getArguments().get(0).getArgument_name().equals(box.getArgument_name()) &&
                                        x.getArguments().get(1).getArgument_id()==carrier_id)){ //risulta vero che on-carrier box carrier_id
                            //Devo vedere se la cassa contiene almeno uno degli elementi richiesti
                            //(has-inside ?b - box ?elem - content ) ; box ?b has content ?elem
                            //Ritorna vero se almeno uno degli elementi richiesti è presente nella box corrente
                            if( state.stream().filter(x -> x.getName().equals("has-inside"))
                                    .anyMatch(x -> x.getArguments().get(0).getArgument_name().equals(box.getArgument_name()) &&
                                            wantedContent.contains(x.getArguments().get(1)))) {
                                return true;
                            }
                        }


                    }

                }

            }

        }


        //Caso 2:
        //Muovere il carrello al deposito quando è pieno è ammissibile ma non è un azione utile
        return false;
    }
    private boolean evaluateMove(Node current, Action op){
        //Caso 1:
        // Move del robot ma la scorsa azione è stata anche una move
        Action previous_op = current.getAction()!=-1 ? problem.getActions().get(current.getAction()) : null;
        if(previous_op!= null && previous_op.getName().equals(op.getName())){
            return false;
        }

        //Caso 2
        // Non ha senso girare a vuoto per la mappa, la move ha senso quando il robot vuole
        // andare in un posto in cui ci sia anche il carrello per prenderlo in mano
        // La move non può essere effettuata se si ha il carrello in mano quindi non risulta necessario controllare tale casistica.
        //L'azione move :parameters
        int[] parameters=op.getParameters(); // [?r - robot, ?from - location , ?to - location)
        int des_id=parameters[2]; //destination
        //Check if exist Carrier at destination des_id in the current state
        List<Predicate> state= getPredicates(current.stream().toArray());
        for(Predicate p: state){
            if(p.getName().equals("at") && p.containsArgByID(des_id)){
                for(Argument carrier:typeToArguments.get("carrier")){
                    String carrier_name=carrier.getArgument_name();
                    if(p.containsArgByName(carrier_name))
                        return true;
                }
            }
        }
        return false;
    }

    public double estimate( Node next, Condition goal){

        double estimated_value=0;


        List<Predicate> next_state= getPredicates(next.stream().toArray());
        List<Predicate> goals= getPredicates(goal.getPositiveFluents().stream().toArray());
        List<Predicate> goals_already_satisfied=getGoalAlreadySatisfied(next_state,goals);
        List<Predicate> goals_not_satisfied_yet=getGoalNotAlreadySatisfied(goals,goals_already_satisfied);

        //Dobbiamo stimare la lontanza dal goal;


        //Dobbiamo effettuare tante azioni ancora quanti sono i goal da soddisfare;
        estimated_value+=goals_not_satisfied_yet.size();
        //Dobbiamo effettuare tante azioni quante sono i posti differenti in cui dobbiamo ancora andare;
        estimated_value+=getPositionsToReach(next_state,goals_not_satisfied_yet);
        //Dobbiamo effettuare tante azioni quanto sono le scatole da caricare sul carrier;
        estimated_value+=getBoxToFillAndToLoad(next_state,goals_not_satisfied_yet);



        return estimated_value;
    }

    private int getPositionsToReach(List<Predicate> next_state,List<Predicate> goals_not_satisfied_yet){
        int postions_to_reach=0;
        //Dobbiamo effettuare tante azioni quante sono i posti differenti in cui dobbiamo ancora andare;
        List<Predicate> robots_positions=next_state.stream().filter(p-> p.getName().equals("at") &&
                typeToArguments.get("robot").contains(p.getArguments().get(0))).collect(Collectors.toList());
        List<Predicate> persons_positions_to_reach=next_state.stream().filter(p-> p.getName().equals("at") &&
                        typeToArguments.get("person").contains(p.getArguments().get(0)))
                .filter(p-> { //Se la persona è presente almeno in un goal non soddisfatto allora dobbiamo raggiungerla
                            Argument person = p.getArguments().get(0);
                            Argument location_p=p.getArguments().get(1);
                            for (Predicate g : goals_not_satisfied_yet)
                                if (g.containsArgByID(person.getArgument_id())) //Ok dobbiamo raggiungere la persona
                                    for(Predicate robot_position:robots_positions) {
                                        //Se c'è un robot in quella posizione però non dobbiamo raggiungerla
                                        Argument location_r=robot_position.getArguments().get(1);
                                        if(location_r!=location_p)
                                            return true;
                                    }

                            return false;
                        }
                )
                .collect(Collectors.toList());
        postions_to_reach=persons_positions_to_reach.size();
        return postions_to_reach;
    }

    private int getBoxToFillAndToLoad(List<Predicate> next_state,List<Predicate> goals_not_satisfied_yet){
        int res=0;
        List<Predicate> goals_not_satisfied=new LinkedList<>(goals_not_satisfied_yet); //La modificheremo quindi creaiamoci una copia

        if(goals_not_satisfied.size()==0){ //Se non vi sono goal da soddisfare non devo caricare alcuna box
            return res;
        }

        //Potenzialmente devo caricare tanti oggetti nelle casse quanti sono gli oggetti che devo ancora consegnare
        res+=goals_not_satisfied.size();

        //Vado a sottrarre il numero di casse contenenti già gli oggetti che devo consegnare.
        res-=next_state.stream().filter(p->{
                boolean check_1=p.getName().equals("has-inside"); //(has-inside ?b ?elem)
                if(!check_1) return false;
                //Elemento nella cassa
                int elem_id=p.getArguments().get(1).getArgument_id();
                if(removeElementByID(goals_not_satisfied,elem_id)) { //Se l'elemento è cotenuto nei goal allora andiamo a sottrarlo
                    return true;
                }
                return false;

        }).count();

        //Quindi potenzialmente devo caricare res casse

        return res;
    }

    private boolean removeElementByID(List<Predicate> predicates ,int element_id){
        Iterator<Predicate> it=predicates.iterator();
        while (it.hasNext()){
            Predicate curr=it.next();
            if(curr.containsArgByID(element_id)){
                it.remove();
                return true;
            }
        }
        return false;
    }




    public double estimate_old(Node current, Node next, Action a, Condition goal) {



        double n_goal_alredy_satisfaied=0;
        double estimated_value=0;

        List<Predicate> new_state= getPredicates(next.stream().toArray());
        List<Predicate> old_state= getPredicates(current.stream().toArray());
        List<Predicate> goals= getPredicates(goal.getPositiveFluents().stream().toArray());
        List<Predicate> goals_already_satisfied=getGoalAlreadySatisfied(new_state,goals);
        List<Predicate> goals_not_satisfied_yet=getGoalNotAlreadySatisfied(goals,goals_already_satisfied);


        estimated_value+=checkBoxes(new_state,goals_already_satisfied,goals_not_satisfied_yet);
        estimated_value+=checkCarrier(new_state,goals_already_satisfied,goals_not_satisfied_yet);


        //Se in questo nodo ho effettuato un azione che mi ha portato a una vicinaza nella soddifazione dei goal allora ritorno subito
        if(getGoalAlreadySatisfied(old_state,goals).size()<goals_already_satisfied.size()) {
            return Double.MIN_VALUE;
        }


        return estimated_value*goals_not_satisfied_yet.size();
    }

    private int countOccurrences(List<Predicate> list,String predicate){
        int count=0;
        for (Predicate p: list){
            if(p.getName().equals(predicate)) count++;
        }
        return count;
    }



    private List<Predicate> getGoalAlreadySatisfied(List<Predicate> predicates,List<Predicate> goals){
        return predicates.stream()
                .filter(goals::contains)
                .collect(Collectors.toList());
    }

    private List<Predicate> getGoalNotAlreadySatisfied(List<Predicate> goals,List<Predicate> goalsAlreadySatisfied){
        return goals.stream()
                .filter(goal -> !goalsAlreadySatisfied.contains(goal))
                .collect(Collectors.toList());
    }


    private double checkCarrier(List<Predicate> predicates, List<Predicate> goalsAlreadySatisfied, List<Predicate> goalsNotSatisfiedYet){
        double numEmptySpacesInCurrentState= countOccurrences(predicates,"empty");
        double c1=5;
        int numOfUnoccupiedCarts=this.n_carrier-countOccurrences(predicates,"is-holding");
        return numEmptySpacesInCurrentState*c1+numOfUnoccupiedCarts;
    }

    private double checkBoxes(List<Predicate> predicates, List<Predicate> goalsAlreadySatisfied, List<Predicate> goalsNotSatisfiedYet) {
        //Il numero di tutte le casse che possono essere full meno il numero delle casse che sono a full in questo stato.
        int numEmptyBoxesInCurrentState = n_boxes-countOccurrences(predicates, "full");

        if (numEmptyBoxesInCurrentState == 0) {
            // If there are no boxes to fill, return -1
            return 0;
        }

        // List of elements contained in the boxes
        List<Argument> boxContents = predicates.stream()
                .filter(p -> "has-inside".equals(p.getName()))
                .map(p -> p.getArguments().get(1))
                .collect(Collectors.toList());

        // List of elements required to achieve the goal
        List<Argument> requiredGoalElements = getListOfElementsRequiredToAchiveTheGoal(goalsNotSatisfiedYet);

        // Number of elements present in requiredGoalElements but not in boxContents: these are the items not yet loaded in the boxes but are needed
        double x1 = requiredGoalElements.stream()
                .filter(arg -> {
                    boolean contains = boxContents.contains(arg);
                    if (contains) {
                        boxContents.remove(arg);
                    }
                    return !contains;
                })
                .count();

        // Number of elements present in boxContents but not in requiredGoalElements: these are the loaded items that are not needed
        double x2 = boxContents.size();

        return (x1*2 + (x2*3))*6;
    }

    /*
    private double checkBoxes(List<Predicate> predicates,List<Predicate> goals_already_satisfied,List<Predicate> goals_not_satisfied_yet){
        int num_full_box_in_current_state=countOccurrences(predicates,"full");
        int num_empty_box_in_current_state=countOccurrences(predicates,"empty");
        if(num_empty_box_in_current_state==0){
            //Se non ci sono casse da riempire ritorna -1
            return -1;
        }

        //Lista degli elementi contenuti nelle casse
        List<Argument> boxContents=predicates.stream()
                .filter(p -> p.getName().equals("has-inside"))
                .map(p -> p.getArguments().get(1))
                .collect(Collectors.toList());
        //Lista degli elementi necessari per raggiungere il goal
        List<Argument> requiredGoalElements=goals_not_satisfied_yet.stream()
                .flatMap(p -> {
                    if ("satisfied-with-at-least-one".equals(p.getName()) && p.getArguments().size() >= 3) {
                        // Se il predicato è "satisfied-with-at-least-one" ci sono due oggetti
                        return Stream.of(
                                p.getArguments().get(1).clone(),
                                p.getArguments().get(2).clone()
                        );
                    } else {
                        // Altrimenti, restituisci solo il secondo argomento
                        return Stream.of(p.getArguments().get(1).clone());
                    }
                })
                .collect(Collectors.toList());
        //Numero di oggetti presenti in required ma non in box of Contents: sarebbero gli oggetti non ancora caricati sulle casse ma che servono
        double x1=0;
        //Numero di oggetti presenti in box of Contents ma non in required: sarebbero gli oggetti caricati ma che non servono
        double x2=0;
        for(Argument arg: requiredGoalElements ){
            if(boxContents.contains(arg)){
                boxContents.remove(arg);
            }
            else x1++;
        }
        //Gli oggetti presenti in boxContents non eliminati allora non erano presenti in required quindi:
        x2=boxContents.size();

        //Se nelle box è presente un elemento che non serve per soddisfare il goal allora dobbiamo mettere un costo elevato
        //Vado a mettere per ogni cibo mancante un costo pari a 1 in più


        return x1+ (x2*2);
    }
    */
}
