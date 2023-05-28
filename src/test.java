import ilog.concert.*;
import ilog.cplex.*;

public class test {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		model1();
	}
	
	public static void model1(){
		try {
			IloCplex cplex = new IloCplex();
			
			IloNumVar x = cplex.numVar(0, Double.MAX_VALUE, "x");
			IloNumVar y = cplex.numVar(0, Double.MAX_VALUE, "y");
			
			IloLinearNumExpr objective = cplex.linearNumExpr();
			objective.addTerm(0.12, x);
			objective.addTerm(0.15, y);
			
			cplex.addMinimize(objective);
			
			cplex.addGe(cplex.sum(cplex.prod(60, x), cplex.prod(60,y)), 300);
			cplex.addGe(cplex.sum(cplex.prod(12, x), cplex.prod(6,y)), 36);
			cplex.addGe(cplex.sum(cplex.prod(10, x), cplex.prod(30,y)), 90);
			
			cplex.solve();
			System.out.println("obj = "+cplex.getObjValue());
			System.out.println("obj = "+cplex.getValue(x));
			System.out.println("obj = "+cplex.getValue(y));
			
		}
		catch(IloException exc) {
			exc.printStackTrace();
		}
	}

}
