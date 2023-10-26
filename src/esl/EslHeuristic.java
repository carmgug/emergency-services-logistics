package esl;

import fr.uga.pddl4j.parser.TypedSymbol;
import fr.uga.pddl4j.problem.Fluent;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.Condition;
import sun.awt.image.ImageWatched;
import utility.Argument;
import utility.Predicate;

import fr.uga.pddl4j.problem.*;


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

        System.out.println(this.idConditionMap);
        System.out.println(problem.getParsedProblem().getObjects());


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


    public double estimate(Node current, Node next, Action a, Condition goal) {

        //Gestire Casistiche in cui non posso determinare la distanza
        if(current.getAction()==-1){ //Se sono al primo nodo non posso stimare la prossima azione almeno per ora;
            return 0;
        }

        //Compare action in old state and in new state.
        Action a1=problem.getActions().get(current.getAction());
        Action a2=a;
        if(a1.getName().equals("move-carrier") && a2.getName().equals("move-carrier") &&
                a1.getInstantiations()[0]==a2.getInstantiations()[0] //Allora un robot ha effettuato due move consecutive
        ) {
            return Double.MAX_VALUE;
        }

        if(a2.getName().equals("release-carrier")){
            return Double.MAX_VALUE;
        }

        if(a1.getName().equals("move") && a2.getName().equals("move")){
            return Double.MAX_VALUE;
        }

        double n_goal_alredy_satisfaied=0;
        double estimated_value=0;

        List<Predicate> new_state= getPredicates(next.stream().toArray());
        List<Predicate> old_state= getPredicates(current.stream().toArray());
        List<Predicate> goals= getPredicates(goal.getPositiveFluents().stream().toArray());
        List<Predicate> goals_already_satisfied=getGoalAlreadySatisfied(new_state,goals);
        List<Predicate> goals_not_satisfied_yet=getGoalNotAlreadySatisfied(goals,goals_already_satisfied);


        estimated_value+=checkBoxes(new_state,goals_already_satisfied,goals_not_satisfied_yet);
        estimated_value+=checkCarrier(new_state,goals_already_satisfied,goals_not_satisfied_yet);


        //System.out.println("Goals: "+goals);
        //System.out.println(getGoalAlreadySatisfied(new_state,goals));
        //System.out.println(getGoalNotAlreadySatisfied(goals,getGoalAlreadySatisfied(new_state,goals)));

        //Se in questo nodo ho effettuato un azione che mi ha portato a una vicinaza nella soddifazione dei goal allora ritorno subito
        if(getGoalAlreadySatisfied(old_state,goals).size()<goals_already_satisfied.size()) {
            return Double.MIN_VALUE;
        }


        return estimated_value;
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
        double c1=2;
        if(numEmptySpacesInCurrentState<goalsNotSatisfiedYet.size()){
            c1=0;
        }
        int numOfUnoccupiedCarts=this.n_carrier-countOccurrences(predicates,"is-holding");


        return 0;
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
        List<Argument> requiredGoalElements = goalsNotSatisfiedYet.stream()
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

        return (x1 + (x2))* goalsNotSatisfiedYet.size();
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
