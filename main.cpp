#include <raylib.h>
#include <cmath>

constexpr int window_width{1280};
constexpr int window_height{1000};
constexpr float g{981.0};
double dt;
Vector2 centerpos;

Color customblue{11,19,53,255};

struct State{
    double x;
    double vx;
    double O;
    double w;

};

State computederivative(const State& state, float M, float m, float L){
    State derived;
    derived.x = state.vx;
    derived.O = state.w;
    float delta = M + m - (m * cos(state.O) * cos(state.O));

    derived.vx = ((M * L * state.w * state.w * sin(state.O)) - (m * g * sin(state.O) * cos(state.O))) / delta;
    derived.w = (-g*(M+m)*sin(state.O)-m*L*state.w*state.w*sin(state.O)*cos(state.O)) / (L * delta);

    return derived;
    
}

void rk4(State& state, float M, float m, float L, double dt){
    State k1 = computederivative(state, M, m, L);
    State s1{
        state.x + 0.5*dt*k1.x,
        state.vx + 0.5*dt*k1.vx,
        state.O + 0.5*dt*k1.O,
        state.w + 0.5*dt*k1.w,
    };
    State k2 = computederivative(s1, M, m, L);
    State s2{
        state.x + 0.5*dt*k2.x,
        state.vx + 0.5*dt*k2.vx,
        state.O + 0.5*dt*k2.O,
        state.w + 0.5*dt*k2.w,
    };
    State k3 = computederivative(s2, M, m, L);
    State s3{
        state.x + dt*k3.x,
        state.vx + dt*k3.vx,
        state.O + dt*k3.O,
        state.w + dt*k3.w,
    };
    State k4 = computederivative(s3, M, m, L);

    state.x += (dt/6.0f) * (k1.x + 2*k2.x + 2*k3.x + k4.x);
    state.O += (dt/6.0f) * (k1.O + 2*k2.O + 2*k3.O + k4.O);
    state.vx += (dt/6.0f) * (k1.vx + 2*k2.vx + 2*k3.vx + k4.vx);
    state.w += (dt/6.0f) * (k1.w + 2*k2.w + 2*k3.w + k4.w);

}

int main(){
    InitWindow(window_width, window_height, "x");
    SetTargetFPS(60);

    float M = 10;
    float m = 2;
    float L = 300;
    centerpos.x = static_cast<float>(window_width) / 2.0f;
    centerpos.y = 0;
    State startstate{0.0, 0.0, 1.0, 0.0};
    int y_offset = 40;
  

    constexpr int rec_width{170};
    constexpr int rec_height{70};

    while(!WindowShouldClose()){
        
        dt = GetFrameTime();
        if (dt > 0.05) dt = 0.05;
        rk4(startstate, M, m, L, dt);
        
        double xd = sin(startstate.O) * L;
        double yd = cos(startstate.O) * L;

        float current_rec_x = centerpos.x + startstate.x;
        float current_rec_y = centerpos.y + y_offset + rec_height;
        Rectangle rec{current_rec_x - rec_width / 2.0f, centerpos.y + y_offset, rec_width, rec_height};
        


        
        BeginDrawing();

        ClearBackground(customblue);
        for (int x{0}; x < window_width; x += 40) {
          DrawLine(x, 0, x, window_height, DARKGRAY);
        }
        for (int y{0}; y < window_height; y += 40) {
          DrawLine(0, y, window_width, y, DARKGRAY);
        }

        DrawRectangleRec(rec, WHITE);
        DrawLine(0, rec.y + rec.height, window_width, rec.y + rec.height, WHITE);
        DrawLine(rec.x+rec_width/2.f, rec.y + rec.height, rec.x+rec_width/2.f + xd, rec.y + rec.height + yd, WHITE);
        DrawCircle(rec.x+rec_width/2.f + xd , rec.y + rec.height + yd , 17,WHITE);

        Vector2 ring_center{rec.x + rec_width / 2.0f, rec.y + rec_height};

        float theta_deg = startstate.O * RAD2DEG;
        float rope_angle = 90.0f - theta_deg;
        float base_angle = (xd >= 0) ? 0.0f : 180.0f;
        float start_a = std::min(base_angle, rope_angle);
        float end_a = std::max(base_angle, rope_angle);

        DrawCircleSectorLines(ring_center, 40.0f, start_a, end_a, 32, RAYWHITE);

        float angle = 90.0f - fabs(theta_deg);
        DrawText(TextFormat("Angle: %.1f", angle),25, window_height - 25, 25, WHITE);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
