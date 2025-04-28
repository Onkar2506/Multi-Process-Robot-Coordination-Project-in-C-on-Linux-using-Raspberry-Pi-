#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/wait.h>
#include <sys/sem.h>
#include <time.h>

#define FIELD_SIZE 30
#define ROBOT_COUNT 3
#define SAFE_DISTANCE 2
#define MOVE_DELAY 200000 // Delay in microseconds (200 ms)

typedef struct {
    int id;
    int x, y;
    int target_x, target_y;
    int speed;
    int active;
} Robot;

// Semaphore for synchronizing access to shared memory
int semid;

// Function to calculate Euclidean distance
double distance(int x1, int y1, int x2, int y2) {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// Function to clear the console
void clear_console() {
    printf("\033[H\033[J");
}

// Wait function for semaphore (lock)
void sem_wait_func(int semid, int sem_num) {
    struct sembuf sb = {sem_num, -1, 0};
    semop(semid, &sb, 1);
}

// Signal function for semaphore (unlock)
void sem_signal_func(int semid, int sem_num) {
    struct sembuf sb = {sem_num, 1, 0};
    semop(semid, &sb, 1);
}

// Display the field with robot positions
void display_field(Robot *robots) {
    char field[FIELD_SIZE][FIELD_SIZE];

    // Initialize the field with empty spaces
    for (int i = 0; i < FIELD_SIZE; i++) {
        for (int j = 0; j < FIELD_SIZE; j++) {
            field[i][j] = ' ';
        }
    }

    // Place robots in the field
    for (int i = 0; i < ROBOT_COUNT; i++) {
        if (robots[i].active) {
            int x = robots[i].x;
            int y = robots[i].y;
            if (x >= 0 && x < FIELD_SIZE && y >= 0 && y < FIELD_SIZE) {
                field[x][y] = '0' + robots[i].id; // Place robot ID on the grid
            }
        }
    }

    // Clear the screen once per update
    clear_console();

    // Print the grid with robots
    printf("+");
    for (int i = 0; i < FIELD_SIZE; i++) printf("--");
    printf("+\n");

    for (int i = 0; i < FIELD_SIZE; i++) {
        printf("|");
        for (int j = 0; j < FIELD_SIZE; j++) {
            printf("%c ", field[i][j]);
        }
        printf("|\n");
    }

    printf("+");
    for (int i = 0; i < FIELD_SIZE; i++) printf("--");
    printf("+\n");

    // Sleep to create smooth movement
    usleep(MOVE_DELAY);
}

// collision avoidance
void move_robot(Robot *self, Robot *robots) {
    if (self->x == self->target_x && self->y == self->target_y) return;

    int dx = self->target_x - self->x;
    int dy = self->target_y - self->y;

    if (dx != 0) dx /= abs(dx); // Normalize the movement direction
    if (dy != 0) dy /= abs(dy); // Normalize the movement direction

    int new_x = self->x + dx;
    int new_y = self->y + dy;

    // Check for collision
    int collision = 0;
    int has_priority = 1;

    for (int i = 0; i < ROBOT_COUNT; i++) {
        if (robots[i].id != self->id && robots[i].active) {
            if (distance(new_x, new_y, robots[i].x, robots[i].y) < SAFE_DISTANCE) {
                collision = 1;
                if (self->id > robots[i].id)
                    has_priority = 0;
            }
        }
    }

    if (collision && !has_priority) {
        usleep((rand() % 400 + 100) * 1000); // Backoff if blocked
        return;
    } else if (collision) {
        // Attempt alternate movement
        int options[4][2] = {{dx, 0}, {0, dy}, {-dx, 0}, {0, -dy}};
        for (int o = 0; o < 4; o++) {
            int nx = self->x + options[o][0];
            int ny = self->y + options[o][1];
            int safe = 1;
            for (int j = 0; j < ROBOT_COUNT; j++) {
                if (robots[j].id != self->id && robots[j].active) {
                    if (distance(nx, ny, robots[j].x, robots[j].y) < SAFE_DISTANCE)
                        safe = 0;
                }
            }
            if (safe) {
                self->x = nx;
                self->y = ny;
                display_field(robots);
                printf("Robot %d avoided and moved to (%d,%d)\n", self->id, nx, ny);
                return;
            }
        }
     
        usleep(500000); // 500ms pause if completely blocked
        return;
    }

    // Safe to move
    self->x = new_x;
    self->y = new_y;
    display_field(robots);
    printf("Robot %d moved to (%d, %d)\n", self->id, new_x, new_y);
}

int main() {
    srand(time(NULL));

    // Create semaphore set
    semid = semget(IPC_PRIVATE, 1, IPC_CREAT | 0666);
    if (semid < 0) {
        perror("semget failed");
        exit(1);
    }

    // Initialize semaphore to 1 (unlocked)
    semctl(semid, 0, SETVAL, 1);

    int shmid = shmget(IPC_PRIVATE, ROBOT_COUNT * sizeof(Robot), IPC_CREAT | 0666);
    if (shmid < 0) {
        perror("shmget failed");
        exit(1);
    }

    Robot *robots = (Robot *)shmat(shmid, NULL, 0);
    if (robots == (Robot *)-1) {
        perror("shmat failed");
        exit(1);
    }

    int start_positions[ROBOT_COUNT][2] = {{0, 0}, {0, FIELD_SIZE - 1}, {FIELD_SIZE - 1, 0}};
    int target_positions[ROBOT_COUNT][2] = {{FIELD_SIZE - 1, FIELD_SIZE - 1}, {FIELD_SIZE - 1, 0}, {0, FIELD_SIZE - 1}};
    int speeds[ROBOT_COUNT] = {1, 1, 1};

    for (int i = 0; i < ROBOT_COUNT; i++) {
        robots[i].id = i;
        robots[i].x = start_positions[i][0];
        robots[i].y = start_positions[i][1];
        robots[i].target_x = target_positions[i][0];
        robots[i].target_y = target_positions[i][1];
        robots[i].speed = speeds[i];
        robots[i].active = 1;
    }

    // Create child processes for each robot
    pid_t pids[ROBOT_COUNT];
    for (int i = 0; i < ROBOT_COUNT; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            while (robots[i].active) {
                sem_wait_func(semid, 0); // Lock the semaphore
                move_robot(&robots[i], robots);
                if (robots[i].x == robots[i].target_x && robots[i].y == robots[i].target_y) {
                    printf("Robot %d reached target!\n", robots[i].id);
                    robots[i].active = 0;
                }
                sem_signal_func(semid, 0); // Unlock the semaphore
                usleep(robots[i].speed * 100000); // Adjust delay based on robot speed
            }
            exit(0);
        } else if (pid < 0) {
            perror("fork failed");
            exit(1);
        } else {
            pids[i] = pid;
        }
    }

    // Parent waits for Robot 2 to reach its target before terminating
    int all_reached = 0;
    while (!all_reached) {
        all_reached = 1;
        for (int i = 0; i < ROBOT_COUNT; i++) {
            if (robots[i].active) {
                all_reached = 0; // Not all robots have reached their target
                break;
            }
        }
        usleep(100000); // Check every 100ms
    }

    // Wait for all child processes to finish
    for (int i = 0; i < ROBOT_COUNT; i++) {
        waitpid(pids[i], NULL, 0);  // Wait for each child process to complete
    }

    // Clean up shared memory and semaphore
    shmdt(robots);
    shmctl(shmid, IPC_RMID, NULL);
    semctl(semid, 0, IPC_RMID);

    printf("Simulation complete.\n");
    return 0;
}
