/*
 * I do not think we actually need this to be in cpp, python works fine.
 * We should focus on putting the slower stuff to cpp first.
 * Tracker and scene.simulate are the slow ones.
 */

# include <string>

class RiskPredictor {
    public: // for now, later make things private

        float horizon_s;
        float step_s;
        float col_tol_x_m;
        float col_tol_y_m;
        float ttc_tol_s;

        RiskPredictor(float H, float step, float col_tol_x, float col_tol_y, float ttc_tol) {
            horizon_s = H;
            step_s = step;
            col_tol_x_m = col_tol_x;
            col_tol_y_m = col_tol_y;
            ttc_tol_s = ttc_tol;
        }

        float getRisk(void* state, string risk_type, int n_sims, bool verbose=false) {
            return 10;
        }
};
