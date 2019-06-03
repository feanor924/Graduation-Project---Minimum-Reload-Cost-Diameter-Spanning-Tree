package bitirme;

/**
 *
 * @author canerbakar
 */

import java.util.*;
import java.lang.*;
import java.io.*;
import java.util.Random;
import org.graphstream.graph.*;
import org.graphstream.graph.implementations.*;
import org.graphstream.algorithm.generator.*;
import static org.graphstream.algorithm.Toolkit.*;

/*--------------------- RANDOM GRAPH ALGORITHM------------*/


class randomgraph
{
   public  int current_edge_weight;                     // used in
							//    next_neighbor
   public int[][] M;					// adjacency matrix to represent a graph
   public boolean[][] M_;
   private int n;					// number of cols/rows
   public int nVerts;					// number of vertices
                              
   private int[] next;                                  // array to track next neighbor

   public randomgraph(int a, int prob, long theseed) 
   {

      double b;			
      int i, j;

      
      n = a;						// initialize n to number of cols/rows
      M = new int[n][n];             			// initialize 2D array to all zeros
      M_ = new boolean[n][n];
      nVerts = n;					// initialize number of vertices
      next = new int[n];                  		// next neighbor for each vertex

      for ( int p = 0; p < nVerts; ++p){
           for (int q = 0; q < nVerts; ++q){
               M_[p][q] = false;
           }
       }
      
      for(i=0; i < nVerts; i++)			// initialize next neighbor
           next[i]=-1;

      Random generator = new Random();
      Random generator2 = new Random(theseed);
      Random mygenerator; 

      if (theseed == -1)
        mygenerator = generator;
      else
        mygenerator = generator2;

      for(i=0; i < nVerts; i++) {
         for(j=0; j < nVerts; j++) {

            if (i == j) 
               M[i][j]=0;
            else if (j < i)
               M[i][j] = M[j][i];
            else {
                  b = mygenerator.nextDouble() * 100;
                  if (b <= prob)
                     M[i][j] = 1;
                  else
                     M[i][j] = 0;                  
                 }
         }
      }


   }

   public void insertVertex(int a, int x, int y)	// insert a vertex
   {
      if(x == y)					// if M[i][i]
      {
         if(a != 0)                                     // if value if not zero, display error and exit
         {
            System.out.println("Cannot initialize graph, M[i][i] must be zero!  Exiting...");
            System.exit(0);
         }
      }

      M[x][y] = a;					// insert vertex into matrix M

   }

   public void display()
   {
    System.out.println("");    				// display the array
    for(int row=0; row<n; row++)
      {
      	for(int col=0; col<n; col++)
      	  System.out.print(M[row][col] + " ");
      	  System.out.println("");
      }
   }

   public int vertices()
   {
      return nVerts;					// return the number of vertices

   }

   public int edgeLength(int a, int b)
   {
      return M[a][b];					// return the edge length

   }

   public int nextneighbor(int v)
   {

      next[v] = next[v] + 1; 				// initialize next[v] to the next neighbor

      if(next[v] < nVerts)
      {
      	while(M[v][next[v]] == 0 && next[v] < nVerts)
      	{
         next[v] = next[v] + 1;                         // initialize next[v] to the next neighbor

        if(next[v] == nVerts)
            break;
      	}

     }

      if(next[v] >= nVerts)
      {
         next[v]=-1;                                    // reset to -1
         current_edge_weight = -1;
      }
      else current_edge_weight = M[v][next[v]];

      return next[v];      				// return next neighbor of v to be processed

   }

    public void resetnext()
    {
        for (int i=0; i < nVerts; i++)	// reset the array next to all -1's
            next[i] = -1;

    }

    public void modifyArray(boolean is){
        
        int counter1= 0;
        Random rand = new Random();
        
       
        for ( int i = 0; i < nVerts; ++i){
            for (int j = 0; j < nVerts; ++j){
                if ( M[i][j] == 1 && M_[j][i] == false){
                    M_[i][j] = true;  
                    int a=0;
                    
                    if ( is == true)
                        a = rand.nextInt(n*2) + 1;
                    else if ( is == false)
                        a = rand.nextInt(n/2) + 1;
                    
                    if ( is == true)
                        counter1 = change(n*2,i,j);
                    
                    if ( counter1 == 0)
                        M[i][j] = a;
                    counter1 = 0;
                }
                else if ( M[i][j] == 1 && M_[j][i] == true ){
                    M[i][j] = M[j][i];
                    M_[i][j] = true;
                }
            }
        }
       
    }
    
    public int change(int counter,int i,int j){
        Random rand = new Random();
        int counter1 = 0,counter2 = 0;
        
        while ( true ){

            int z = rand.nextInt(counter) + 1;

            for (int l = 0; l < nVerts; ++l){

                if ( M[i][l] == z){
                    ++counter2;
                }
            }

            for (int l = 0; l < nVerts; ++l){

                if ( M[l][j] == z ){
                    ++counter2;
                }
            }
            
            
            if ( counter2 == 0){
                ++counter1;
                M[i][j] = z;
                break;
            }
            counter2 = 0;
        }
        return counter1;
        
    }



}


    /*-----------------FIND SHORTEST PATH BETWEEN 2 LEAF -----*/


class MyLinkedList extends LinkedList<Integer> {
}

class MST
{
    // Number of vertices in the graph
    public int V;
    
    public int [][]neighbour;
    
    public int cnt1 = 0;
    
    public int [][] pairies;
    
    public int size1;
 
    public int size2;
    
    public int [][] u_;
    
    private LinkedList<Integer> adj[];
    
    public ArrayList<Integer> a = new ArrayList<Integer>();

           
    MST(){} 
    
    MST(int v){
 
        this.V = v;
        
        adj = new LinkedList[V]; 
        pairies = new int [V][V];
        u_ = new int [V*2][V*2];
        neighbour = new int [V*40][V*40];



        for (int i=0; i<V; ++i){
            adj[i] = new LinkedList();
        }

        
        for (int i = 0; i < V*2; i++){
            for ( int j = 0; j < V*2; ++j){
                u_[i][j] = -1;
            }
        }
            
        
    }
 
    public void addEdge(int v, int w)
    {
        adj[v].add(w); // Add w to v's list.
    }
    

	
    public void printAllPathsUtil(int v, int d, boolean visited[],ArrayList<Integer> path)
    {
        // Mark the current node as visited and print it
        visited[v] = true;
        path.add(v);


        if (v == d) {
            for (int p : path) {
               a.add(p);
            } 
            a.add(-1);

        }
        else{
            // Recur for all the vertices adjacent to this vertex
            Iterator<Integer> i = adj[v].listIterator();
            while (i.hasNext())
            {
                int n = i.next();
                if (!visited[n])
                    printAllPathsUtil(n, d, visited, path);
            }
        }
        path.remove(new Integer(v));
        visited[v] = false;
    }
	

	
    public void printAllPaths(int v, int u)
    {
        // Mark all the vertices as not visited(set as
        // false by default in java)
        boolean visited[] = new boolean[V];
        ArrayList<Integer> path = new ArrayList<Integer>();

        // Call the recursive helper function to print DFS traversal
        printAllPathsUtil(v, u, visited, path);
    }
    
    
    
    /* -------------------ALGORITHM START HERE ------------------------*/
    /*-----------------------------------------------------------------*/
    
    
    
    
    
    // A utility function to find the vertex with minimum key
    // value, from the set of vertices not yet included in MST
    public int[] minKey(boolean is,int key[][][], Boolean mstSet1[],int count_,int input){
        // Initialize min value
        int min = Integer.MAX_VALUE;
        int mins[] = new int[3];
        Random rand = new Random();
        
        if ( is == true){
            for (int v1 = 0; v1 < V; v1++){         
                for (int v = 0; v < V; v++){                                
                    if ( mstSet1[v] == false){
                        for ( int v_=0; v_ < V; ++v_){
                            if ( mstSet1[v_] == false &&  key[v1][v][v_] < min ){                        

                                min = key[v1][v][v_];
                                mins[0] = v1;
                                mins[1] = v;
                                mins[2] = v_;
                            }

                        }
                    }
                }
            }
        }
        else if ( is == false){
            
                
                
            int a_0 = rand.nextInt(input);   
                
                
            for (int v1 = 0; v1 < V; v1++){         
                for (int v = 0; v < V; v++){
                    if ( mstSet1[v] == false){
                        for ( int v_=0; v_ < V; ++v_){
                            if ( mstSet1[v_] == false &&  key[v1][v][v_] != Integer.MAX_VALUE && ( key[v1][v][v_] > a_0 ) ){                        

                                mins[0] = v1;
                                mins[1] = v;
                                mins[2] = v_;
                                return mins;

                            }
                        }
                    }
                }
            }
                


        }
        
        return mins;
    }


    public int primMST(boolean is,int graph[][],int colours[][],int key [][][],int count_,int input)
    {

       
        
        ArrayList<ArrayList<Integer>> outer = new ArrayList<ArrayList<Integer>>();
              
        
        for(int i = 0; i < V; i++)
            outer.add(new ArrayList<Integer>());
 

        Boolean mstSet1[] = new Boolean[V];
 
        // Initialize all keys as INFINITE
        for (int i = 0; i < V; i++){
            for ( int j = 0; j < V; ++j){

                mstSet1[i] = false;
            }
        }


        
        int x = 0, y = 0;
        
        
        
        for (int t = 0; t < V*40; t++)
            for ( int w = 0; w < V*40; ++w)
                neighbour[t][w] = -1;
        
        

        int i=0;
  
        for (int count = 0; count < V; count++)
        {               
            int u[] = minKey(is,key,mstSet1,count_,input);

            mstSet1[u[1]] = true;
            mstSet1[u[2]] = true;
            

            if ( count > 0){
                if ( count > 1){
                    y=1;
                    u_[x][y-1] = u[0];
                    u_[x][y++] = u[1];
                    u_[x][y] = u[2];
                    ++x;
                }
                else if ( count == 1){
                    u_[x][y++] = u[1];
                    u_[x][y] = u[2];
                    ++x;
                }
            }
            else{
                u_[x][y++] = u[1];
            }

            int j=0;
            
            for (int v = 0; v < V; v++){
                               
                if ( graph[u[1]][v] != 0  )
                {
                    neighbour[i][j] = v;
                    ++j;
               
                    for (int z = 0; z < V; z++){
                        if ( graph[v][z] != 0  )
                        {
                            neighbour[i][j] = z;
                            ++j;
                        }  
                    }
                    j=0;
                    ++i;
                }
            }

            calculateDistance(is,neighbour,key,graph,colours,u[1]);  
        }
        
        
        size1 = x-1;
        size2 = y+1;
        
       
        printMST();
        
        ArrayList<Integer> listLeaf = determineLeaf(u_,x-1,y+1);

        int co_ = 0;
        int z = 0;
                    
        for ( int m = 0; m < V; ++m){
            for ( int j = 0; j < V; ++j){
                for (int k = 0; k< V;++k){
                    if ( u_[j][k] == m){
                        if ( k == 0){
                            for ( int p = 0; p < outer.get(m).size(); ++p){
                                if ( outer.get(m).get(p).intValue() == u_[j][k+1])
                                   ++co_;
                            }
                            if ( co_ == 0)
                                outer.get(m).add(u_[j][k+1]);
                            co_=0;
                        }
                        else if ( k == 2){
                            for ( int p = 0; p < outer.get(m).size(); ++p){
                                if ( outer.get(m).get(p).intValue() == u_[j][k-1])
                                   ++co_;
                            }
                            if ( co_ == 0)
                                outer.get(m).add(u_[j][k-1]);
                            co_=0;
                        }
                        else if ( k == 1 ){ 
                            for ( int p = 0; p < outer.get(m).size(); ++p){
                                if ( outer.get(m).get(p).intValue() == u_[j][k-1])
                                   ++co_;
                            }
                            if ( co_ == 0)
                                outer.get(m).add(u_[j][k-1]);
                            co_=0;
                            
                            for ( int p = 0; p < outer.get(m).size(); ++p){
                                if ( outer.get(m).get(p).intValue() == u_[j][k+1])
                                   ++co_;
                            }
                            if ( co_ == 0)
                                outer.get(m).add(u_[j][k+1]);
                            co_=0;
        
                        }                        
                    }
                    z = 0;
                }
            }                    
        }


        for ( int p = 0; p < outer.size(); ++p){
            for ( int q = 0; q < outer.get(p).size(); ++q){
                if ( outer.get(p).get(q).intValue() != -1 ){
                    
                    if ( p != 0 || outer.get(p).get(q).intValue() != 0){
                        addEdge(p,outer.get(p).get(q).intValue());
                    }
             
                }
            }
        }
        
    

        int max = calculateMaxCost(listLeaf,graph,colours);
        
        return max;

        
       
    }
 
    public void calculateDistance(boolean is,int neighbour [][],int key[][][],int graph[][],int colours[][],int u){

        
        for ( int i = 0; i < V*40; ++i){
            for ( int j = 1; j < V*40; ++j){
                if ( neighbour[i][j] != -1 && graph[neighbour[i][0]][neighbour[i][j]] > 0 && graph[u][neighbour[i][0]] > 0
                        && graph[neighbour[i][0]][neighbour[i][j]] != graph[u][neighbour[i][0]] ){                                        
                    if ( is == true && colours[graph[u][neighbour[i][0]]-1][graph[neighbour[i][0]][neighbour[i][j]]-1] < key[u][neighbour[i][0]][neighbour[i][j]] ){
                        
                        key[u][neighbour[i][0]][neighbour[i][j]] = colours[graph[u][neighbour[i][0]]-1][graph[neighbour[i][0]][neighbour[i][j]]-1];

                    }
                    else if ( is == false ){
                        key[u][neighbour[i][0]][neighbour[i][j]] = colours[graph[u][neighbour[i][0]]-1][graph[neighbour[i][0]][neighbour[i][j]]-1];

                    }
                }
            }            
        }

    }
    
    public int calculateMaxCost(ArrayList<Integer> listLeaf,int graph[][],int colours[][]){
        
        ArrayList<Integer> listLeaf_ = new ArrayList<Integer>();        
        ArrayList<Integer> nodes = new ArrayList<Integer>();
        ArrayList<Integer> nodes_ = new ArrayList<Integer>();
        
        for ( int i = 0; i< listLeaf.size(); ++i){
            for ( int j = 0; j < listLeaf.size(); ++j){
                if( listLeaf.get(i).intValue() != listLeaf.get(j).intValue() ){
                    printAllPaths(listLeaf.get(i).intValue(),listLeaf.get(j).intValue());


                    
                    for ( int k = 0; k < a.size(); ++k){
                        
                        if ( a.get(k).intValue() == -1  ){
                            nodes.add(calculateCost(graph,nodes_,colours,listLeaf.get(i).intValue(),listLeaf.get(j).intValue()));
                            nodes_.clear();
                        }
                        else{
                            nodes_.add(a.get(k).intValue());
                        }
                        
                    }

                    int min = Integer.MAX_VALUE;
                    
                    for ( int k = 0; k < nodes.size(); ++k){
                        if ( nodes.get(k).intValue() < min ){
                            min = nodes.get(k).intValue();
                        }
                    }
                    
                    listLeaf_.add(min);
                    
                    nodes_.clear();
                    nodes.clear(); 
                    a.clear();
                    
                }
            }
        }
        
        int max = 0;
        
        for ( int i=0; i < listLeaf_.size(); ++i){
            if ( listLeaf_.get(i).intValue() > max)
                max = listLeaf_.get(i).intValue();
        }
        
        if ( listLeaf.size() == 1){
            max = 4;
        }
        

        return max;
       
    }
    
    public int calculateCost(int graph[][],ArrayList<Integer> list,int colours[][],int leaf1,int leaf2){
        
        int color1 = 0;
        int cost = 0;
       
        
        list.remove(0);
        list.remove(list.size()-1);

        
        if (  list.size() > 1){
            int color = graph[leaf1][list.get(0).intValue()];
            color1 = graph[list.get(0).intValue()][list.get(1).intValue()];
            
            if ( color == 0 || color1 == 0){
                System.out.println("asdasdasdasdasdsa1");
                cost += 1;  
            }
            else{
                
                cost += colours[color-1][color1-1];
            }
            
            for ( int i = 0; i < list.size()-2; ++i){
                color = graph[list.get(i).intValue()][list.get(i+1).intValue()];
                
                color1 = graph[list.get(i+1).intValue()][list.get(i+2).intValue()];
                if ( color == 0 || color1 == 0){
                    System.out.println("asdasdasdasdasdsa2");
                    cost += 1;  
                }
                else{
           
                    cost += colours[color-1][color1-1];
                }
                
            }
            
            color = graph[list.get(list.size()-2).intValue()][list.get(list.size()-1).intValue()];
            color1 = graph[list.get(list.size()-1).intValue()][leaf2];
            if ( color == 0 || color1 == 0){
                System.out.println("asdasdasdasdasdsa3");
                cost += 1;  
            }
            else{
                
                cost += colours[color-1][color1-1];
            }
        }
        else if ( list.size() == 1){
            int color = graph[leaf1][list.get(0).intValue()];
            color1 = graph[list.get(0).intValue()][leaf2];        
            if ( color == 0 || color1 == 0){
                System.out.println("asdasdasdasdasdsa4");
                cost += 1;  
            }
            else{
              
                cost += colours[color-1][color1-1];
            }
            
        }
        else {
            System.out.println("List get empty");
            cost += 0;
        }


        return cost;
    }
    
     
    
    public void printMST()
    {
        int cnt = 0;       
        
        for (int i = 0; i < size1; i++){
            for (int j = 0; j < size2; j++){
                if ( u_[i][j] == 0 ){
                    ++cnt;
                }                
            }
            if ( cnt < 2){
                ++cnt1;
                System.out.println(u_[i][0] + " - " + u_[i][1]+ " - " + u_[i][2]);
            }
            cnt = 0;
        }
        size1 = cnt1;
        
        System.out.println("-------------------------");
    }
    
    
    public ArrayList<Integer> determineLeaf(int parent[][], int x,int y){
        
        ArrayList<Integer> listLeaf = new ArrayList<Integer>();
        int leaf_[] = new int [ V * 2];
        int z = 0;
        int count = 0;
        
        for ( int i = 0; i < x; ++i){
            leaf_[z++] = parent[i][0];
            leaf_[z++] = parent[i][2];
        }
        
        for ( int i = 0; i < z;++i){
            for ( int j = 0; j < x; ++j){
                for ( int k = 0; k < y; k++){                    
                    if ( leaf_[i] == parent[j][k])
                        ++count;
                }   
            }
            if ( count == 1)
                listLeaf.add(leaf_[i]);
            count = 0;
        }

        return listLeaf;
    }


    
}


    /*---------------------- COLORING GRAPH ------------------------*/


class GeneracolorRGB {

    static Random randomGenerator;

    static {
        randomGenerator = new Random();
    }


    public static void main(String[] args) {
        for (int i = 0; i < 100; i++) {
            System.out.println(generateColor());
        }
    }

    private static String generateColor() {
        int newColor = 0x1000000 + randomGenerator.nextInt(0x1000000);
        return "#" + Integer.toHexString(newColor).substring(1, 7);
    }
}


public class Bitirme {

    /**
     * @param args the command line arguments
     */
    
    public int graph_[][];
    public int algoGraph_[][];
    public int randomGraph_[][];
    public int algoVertex;
    public int randomVertex;
    public int Vertex;
    public String arrr [][];
    public int cost1;
    public int cost2;
    
    public void foo(int input){
        
        randomgraph graphMatrix = new randomgraph(input, 30, -1);
        arrr = new String [input][input];
        
        if ( input > 0 && input < 11){
            graphMatrix = new randomgraph(input, 50, -1);
        }
        else if ( input > 10 && input < 21){
            graphMatrix = new randomgraph(input, 40, -1);
        }
        else{
            graphMatrix = new randomgraph(input, 20, -1);
        }
        
        graphMatrix.modifyArray(true);
        
                
        graphMatrix.display();
        
        System.out.println("---------------------");
        
        graph_       = new int [input][input];
        algoGraph_   = new int [input][input];
        randomGraph_ = new int [input][input];
        
        for (int k = 0; k < input; k++){                    
            for (int i = 0; i < input; i++){
                graph_[k][i] = graphMatrix.M[k][i];
            }
        }
        
        Vertex = input;
        
        
        
        
        
        randomgraph colourMatrix = new randomgraph(input*2, 100, -1);
        
        colourMatrix.modifyArray(false);
        
        colourMatrix.display();
        
        System.out.println("---------------------");
        
        int key[][][] = new int [input][input][input];
        

        for (int k = 0; k < input; k++){                    
            for (int i = 0; i < input; i++){
                for ( int j = 0; j < input; ++j){
                    key[k][i][j] = Integer.MAX_VALUE;
                }
            }
        }

        
        int m[] = new int[5];
        MST []ms_ = new MST[5];
          

        for ( int i = 0; i < 5; ++i){
            
            for (int k = 0; k < input; k++){                    
                for (int i_ = 0; i_ < input; i_++){
                    for ( int j = 0; j < input; ++j){
                        key[k][i_][j] = Integer.MAX_VALUE;
                    }
                }
            }
            key[i][i][i] = 0;
            
            MST t_1 = new MST(input);
            ms_[i] = t_1;  
            m[i] = t_1.primMST(true,graphMatrix.M,colourMatrix.M,key,i,input);
        
        }
        
        int min = Integer.MAX_VALUE;
        int indx = 0;
        
        for ( int i = 0; i < 5; ++i){
            if ( m[i] < min ){
                min = m[i];
                indx = i;
            }
        }

        System.out.println("Total cost: " + m[indx]);
        cost1 = m[indx];
        
        for (int k = 0; k < input; k++){                    
            for (int i = 0; i < input; i++){
                algoGraph_[k][i] = ms_[indx].u_[k][i];
            }
        }

        algoVertex = ms_[indx].size1;
        
        
        System.out.println("-------------------------------------------------");
        

   
                   
        MST t_2 = new MST(input);
        
                
        for (int k = 0; k < input; k++){                    
            for (int i = 0; i < input; i++){
                for ( int j = 0; j < input; ++j){
                    key[k][i][j] = Integer.MAX_VALUE;
                }
            }
        }


        key[0][0][0] = 0;     



        int m_0 = t_2.primMST(false,graphMatrix.M,colourMatrix.M,key,0,input);
        
        
        for (int k = 0; k < input; k++){                    
            for (int i = 0; i < input; i++){
                randomGraph_[k][i] = t_2.u_[k][i];
            }
        }

        randomVertex = t_2.size1;


        System.out.println("Random Total cost: " + m_0);
        
        cost2 = m_0;
        
        
  
    }
    

    
    public Bitirme(){
        
    }
    
    
    private static String generateColor() {
        Random randomGenerator = new Random();;
        int newColor = randomGenerator.nextInt(0x1000000);
        return String.format("#%06X", newColor);
    }
    
    public static int [] HexToColor(String hex) 
    {
        int arr[] = new int[3];
        hex = hex.replace("#", "");
        

      
        arr[0] = Integer.valueOf(hex.substring(0, 2), 16);
        arr[1] = Integer.valueOf(hex.substring(2, 4), 16);
        arr[2] = Integer.valueOf(hex.substring(4, 6), 16);
        return arr;

    }
    
    
    
    public void Bitirme1(boolean is,int [][] u_,int x, int input){
        
        
        Graph graph = new SingleGraph("tutorial 1");
        graph.addAttribute("ui.stylesheet", styleSheet);
        
        graph.setAutoCreate(true);
        graph.setStrict(false);
        graph.display();
        
        
        
 
    
        
        if ( is == false ){
            
            for ( int i = 0; i < x; ++i){
                if ( i == 0){
                    String s1 = Integer.toString(u_[i][0]);
                    String s2 = Integer.toString(u_[i][1]);
                    String s3 = Integer.toString(u_[i][2]);
                    graph.addNode(s1);
                    graph.addNode(s2);
                    graph.addNode(s3);
                    graph.addEdge(s1 + "_" + s2, s1, s2);
                    graph.addEdge(s2 + "_" + s3, s2, s3); 
                    
                }
                else{
  
                    int cnt = 0;
                    for ( int j = 0; j < 3; ++j){
                        for(Node n: graph.getEachNode() ) {
                            String s1 = Integer.toString(u_[i][j]);
                            if ( n.getId() != s1){
                                graph.addNode(s1);
                                if ( j > 0 ){
                                    String s2 = Integer.toString(u_[i][j-1]);
                                    graph.addEdge(s1 + "_" + s2, s1, s2);
                                }
                            }
          
                        }
                    }
                    
                }
                
            }
            

            
        }
        else {
            int cnt = 0;
            for ( int i = 0; i < x; ++i ){
                for ( int j = 0; j < x; ++j ){
                    if ( u_[i][j] != 0 ){
                        if ( cnt == 0){
                            String s1 = Integer.toString(i);
                            String s2 = Integer.toString(j);
                            graph.addNode(s1);
                            graph.addNode(s2);
                            graph.addEdge(s1 + "_" + s2, s1, s2);
                            ++cnt;
                        }
                        else{
                            for(Node n: graph.getEachNode() ) {
                                String s1 = Integer.toString(i);
                                String s2 = Integer.toString(j);
                                if ( n.getId() != s1){
                                    graph.addNode(s1);
                                    graph.addEdge(s1 + "_" + s2, s1, s2);
                                }
                                else if ( n.getId() != s2){
                                    graph.addNode(s2);
                                    graph.addEdge(s2 + "_" + s1, s2, s1);
                                }
                            }
                        }
                    } 
                            
                }
            }
        }
               
        int arr1[][] = new int [input][input];
        
        for ( int i = 0; i < input; ++i ){
            for ( int j = 0; j < input; ++j ){
                arr1[i][j] = -1;
            }
        }
        
        for(Node n: graph.getEachNode() ) {
            n.addAttribute("ui.label", n.getId());
            for ( Edge e: n.getEachEdge()){
                
                
                
                String s2 = e.getId();
                StringBuilder s1_ = new StringBuilder();
                StringBuilder s2_ = new StringBuilder();
                int i = 0;
                
                while ( true ){
                    if ( s2.charAt(i) == '_' ){
                        ++i;
                        break;
                    }
                    s1_.append(s2.charAt(i));
                    ++i;
                }
                
                for ( int j = i; j < s2.length(); ++j){
                    s2_.append(s2.charAt(j));
                }
                
                if ( arr1[Integer.parseInt(s1_.toString())][Integer.parseInt(s2_.toString())] == -1){
                    arr1[Integer.parseInt(s1_.toString())][Integer.parseInt(s2_.toString())] = 0;

                
                    int s1__ = 0, s2__ = 0, s3__ = 0;

                    if ( Integer.parseInt(s1_.toString()) < Integer.parseInt(s2_.toString()) ){

                        if ( is == true){
                            
                            int arr[] = new int[3];
                            arr = HexToColor(generateColor());
                            String s = Integer.toString(arr[0]) + "-" + Integer.toString(arr[1]) + "-" + Integer.toString(arr[2]) ;

                   

                            arrr[Integer.parseInt(s1_.toString())][Integer.parseInt(s2_.toString())] = s;

                            s1__ = arr[0];
                            s2__ = arr[1];
                            s3__ = arr[2];
                        }
                        else if ( is == false){
            
                            String s = arrr[Integer.parseInt(s1_.toString())][Integer.parseInt(s2_.toString())];
                    
                            String[] value_split = s.split("-");
                            s1__ = Integer.parseInt(value_split[0]);
                            s2__ = Integer.parseInt(value_split[1]);
                            s3__ = Integer.parseInt(value_split[2]);
                        }
                    }
                    else{
                       
                        String s = arrr[Integer.parseInt(s2_.toString())][Integer.parseInt(s1_.toString())];
                
                        if ( s != null ){
                            String[] value_split = s.split("-");
                            s1__ = Integer.parseInt(value_split[0]);
                            s2__ = Integer.parseInt(value_split[1]);
                            s3__ = Integer.parseInt(value_split[2]);
                        }
                    }





                    String s1___ = Integer.toString(s1__) + ",";
                    String s2___ = Integer.toString(s2__) + ",";
                    String s3___ = Integer.toString(s3__);

                    e.addAttribute("ui.style", "fill-color: rgb(" + s1___ + s2___ + s3__ + ");");
                }
            }

        }
        
        
    }

    
    
    protected String styleSheet =
        "node {                                 " +
        "	fill-color: black;              " +
        "       size: 20px;                     " +
        "       text-mode:normal;               " + 
        "       text-style: bold;               " +    
        "       text-background-mode: plain;    " +
        "       fill-mode: dyn-plain;           " +
        "       text-visibility-mode: normal;   " +
        "       text-size:25px;                 " +
        "       text-color:red;                 " +
        "}                                      " +  
        "edge {                                 " +
        "       shape: line;                    " +
        "       size: 3px;                      " +
        "}" 
        ;
    
    
}



