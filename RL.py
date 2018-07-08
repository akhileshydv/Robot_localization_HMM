import numpy as np
import matplotlib.pyplot as pl
import matplotlib.pyplot as pa

X=4;Y=16    # shape of grid
grid=np.ones([4,16])  
T=np.zeros([42,42],)  #transition matrix
empty_sq=[]  # empty locations in the grid
#epsilon=0.4
O=np.zeros([42,42]) # emission matrix

# find empty squares in the grid
def init_empty_sq():
    for x in range(0,4):
        for y in range(0,16):
            if(grid[x][y]==1):
                empty_sq.append((x,y))

#calculate transition model 
def trans_model():
    for i in range(0,42):
        (x ,y)=empty_sq[i]
        Neigh=Neighbour(x,y)
        Ns=len(Neigh)
        for j in range(0,Ns):
            index=empty_sq.index(Neigh[j])
            T[i][index]=1/Ns

#calculate sensor model at time t            
# (x,y) is true location of the robot at time t
def sensor_model(x,y,epsilon):
    R=sensor_reading(x,y)
    for i in range(0,42):
        (a,b)=empty_sq[i]
        A=sensor_reading(a,b)
        dit=error(A,R)
        t=np.power(1-epsilon,4-dit)*np.power(epsilon,dit)
        O[i][i]=t;
            
# find all possible neighbour of a current location
def Neighbour(x,y):
    a_list=[]
    if x-1>=0 and grid[x-1][y]==1:
        a_list.append((x-1,y))    
    if x+1<X and grid[x+1][y]==1:
        a_list.append((x+1,y)) 
    if y+1<Y and grid[x][y+1]==1:
        a_list.append((x,y+1)) 
    if y-1>=0 and grid[x][y-1]==1:
        a_list.append((x,y-1))
    return a_list    

#find sensor reading given a location 
def sensor_reading(x,y):
    reading=list("1111")    
    if x-1<0 or grid[x-1][y]==0:
        reading[0]="0"    
    if x+1>=X or grid[x+1][y]==0:
        reading[1]="0"
    if y+1>=Y or grid[x][y+1]==0:
        reading[2]="0"
    if y-1<0 or grid[x][y-1]==0:
        reading[3]="0"    
    return reading

# error is discrepancy between true value of a square and sensor reading
def error(A,R):
    cnt=0;
    for i in range(0,4):
        if A[i]!= R[i]:
            cnt=cnt+1        
    return cnt

#filtering
def filtering(f):
    f=np.matmul(O,np.matmul(np.transpose(T),f))
    return f
#create the grid    
def initialize_grid():
    grid[0][4]=0;grid[0][10]=0;grid[0][14]=0;grid[1][0]=0
    grid[1][1]=0;grid[1][4]=0;grid[1][6]=0;grid[1][7]=0
    grid[1][9]=0;grid[1][11]=0;grid[1][13]=0;grid[1][14]=0
    grid[1][15]=0;grid[2][0]=0;grid[2][4]=0;grid[2][6]=0
    grid[2][7]=0;grid[2][13]=0;grid[2][14]=0;grid[3][2]=0
    grid[3][6]=0;grid[3][11]=0
    '''
    for i in range(0,4):
        for j in range(0,16):
            print("("+str(i)+","+str(j)+")="+str(int(grid[i][j])), end =" ")
        print("")    
    '''
def HMM():
    start=1
    f=np.full([42,1],1/42) #prior
    m=np.full([42,1],1/42)
    arr_err=[]
    e_path=[]
    a_path=[]
    for i in range(0,42):
        if start==1:
           print("Choose state to start:")
           start=0
        else:
           print("Choose state to move:")  
        a=int(input("Enter x:"))
        b=int(input("Enter y:"))
        if(a<0 or a>4 or b<0 or b>15 or grid[a][b]==0):
            continue
        sensor_model(a,b)           # calculate O matrix 
        f=filtering(f)              #find posterior
        size=len(f)
        #print(O)
        #print(f)
        ind=np.argmax(f)
        #print("ind="+str(ind))
        print("HMM says:"+str(empty_sq[ind]))
        x,y=empty_sq[ind]
        arr_err.append(abs(a-x)+abs(b-y));
        m,z=viterbi_algo(m)
        print (str(m)+" "+ str(z))
        e_path.append(empty_sq[z])
        a_path.append((a,b))
        print(e_path)
    acc=path_acc(e_path,a_path)
    print("accuracy="+str(acc))

def HMM_400():
    sigma_val=[0.0,0.02,0.05,0.1,0.2,0.4]
    _run=400
    _localization_error=[]
    _path_acuracy=[]
    for _error in range(0,len(sigma_val)):
        epsilon=sigma_val[_error]
        avg_arr_err=[]
        avg_arr_acc=[]
        for iter in range(0,_run):
            f=np.full([42,1],1/42) #prior
            m=np.full([42,1],1/42)
            arr_err=[]
            arr_acc=[]
            e_path=[]
            a_path=[]
            while(True):
                ind=np.random.randint(0,42, size=None)
                (a,b)=empty_sq[ind]
                N=Neighbour(a,b)
                if(len(N)!=0):
                    break;
            start=1
            for i in range(0,42):
                if start!=1:
                    N=Neighbour(a,b)
                    Nind=np.random.randint(0,len(N), size=None)
                    (a,b)=N[Nind]        
                sensor_model(a,b,epsilon)           # calculate O matrix 
                f=filtering(f)              #find posterior
                ind=np.argmax(f)
                #print("ind="+str(ind))
                x,y=empty_sq[ind]
                arr_err.append(abs(a-x)+abs(b-y));
                m,z=viterbi_algo(m)
                e_path.append(empty_sq[z])
                a_path.append((a,b))
                acc=path_acc(e_path,a_path)
                arr_acc.append(acc)
                start=0;
            avg_arr_err.append(arr_err)
            avg_arr_acc.append(arr_acc)    
            #print(iter)    
            #print(arr_err)
            #print(arr_acc)         
        cal_avg_err=[]
        cal_avg_acc=[]
        for i in range(0,42):
            cal_avg_err.append(0)
            cal_avg_acc.append(0)
        for i in range(0,_run):
            for j in range(0,42):
                cal_avg_err[j]=cal_avg_err[j]+avg_arr_err[i][j]
                cal_avg_acc[j]=cal_avg_acc[j]+avg_arr_acc[i][j]
        for i in range(0,42):
            cal_avg_err[i]=cal_avg_err[i]/_run
            cal_avg_acc[i]=cal_avg_acc[i]/_run
        #'''    
        
        _localization_error.append(cal_avg_err)
        _path_acuracy.append(cal_avg_acc)
        
        #'''
        
        '''
        pa.title('Accuracy')        
        pa.xlabel('Number of Observations')
        pa.ylabel('Path Accuracy')
        pa.plot(cal_avg_acc,label=sigma_val[_error])       
        
        '''
    #print(_localization_error)
    #print(_path_acuracy)
    pl.title('Error')        
    pl.xlabel('Number of Observations')
    pl.ylabel('Localization error')
    for i in range(0,len(sigma_val)):
        pl.plot(_localization_error[i],label=sigma_val[i])
    pl.legend()    
    pl.show()
    
    pl.title('Accuracy')        
    pl.xlabel('Number of Observations')
    pl.ylabel('Path Accuracy')
    for i in range(0,len(sigma_val)):
        pl.plot(_path_acuracy[i],label=sigma_val[i])
    pl.legend()    
    pl.show()    
        
               
    
    

def path_acc(e_path,a_path):
    cnt=0;
    for i in range(0,len(e_path)):
         if e_path[i]==a_path[i]:
             cnt=cnt+1;
    return cnt/len(e_path)
    

#viterbi algorithm to find most likely path
def viterbi_algo(m):
    g=np.multiply(np.transpose(T),np.transpose(m))
    #print(np.transpose(T))
    #print(np.transpose(m))
    t=np.amax(g,axis=1)
    #print(t)
    m=np.matmul(O,t)
    max_ind=np.argmax(m)
    return m,max_ind
    
    
if __name__ == '__main__':
    initialize_grid()
    init_empty_sq()
    trans_model()
    TT=np.transpose(T)
    HMM_400()             
