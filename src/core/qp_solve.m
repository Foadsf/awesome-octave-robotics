% qp_solve.m — Active-set quadratic program solver (no Octave Forge).
%
%   [x, obj, info] = qp_solve(H, f, A, b, Aeq, beq, lb, ub, options)
%
%   min  1/2 x' H x + f' x
%   s.t. A x <= b,  Aeq x = beq,  lb <= x <= ub
%
%   Empty matrices are allowed for any constraint set. Bounds are converted
%   to inequality rows; equalities are imposed exactly in every QP subproblem
%   (no doubled rows, so the KKT system stays non-singular). Uses an
%   active-set method over the inequality constraints.
%
%   info.status  : 'optimal' | 'maxiter' | 'infeasible-start'
%   info.iter    : iterations
%   info.lambda  : Lagrange multipliers for the inequality constraints

function [x, obj, info] = qp_solve(H, f, A, b, Aeq, beq, lb, ub, options)
  if nargin < 3, A = []; end
  if nargin < 4, b = []; end
  if nargin < 5, Aeq = []; end
  if nargin < 6, beq = []; end
  if nargin < 7, lb = []; end
  if nargin < 8, ub = []; end
  if nargin < 9 || isempty(options), options = struct(); end
  if isstruct(options) && isfield(options, 'maxiter'), maxit = options.maxiter; else maxit = 500; end
  if isstruct(options) && isfield(options, 'tol'),     tol     = options.tol;     else tol     = 1e-8; end
  if isstruct(options) && isfield(options, 'eqtol'),   eqtol   = options.eqtol;   else eqtol   = 1e-8; end
  n = length(f);
  f = reshape(f, n, 1);
  H = 0.5 * (H + H') + 1e-8 * eye(n);  % symmetrize + regularize
  if ~isempty(Aeq), beq = reshape(beq, [], 1); end

  x = zeros(n, 1); obj = 0; info = struct();

  % ---------- build inequality system C x <= d (includes bounds) ----------
  C = []; d = [];
  if ~isempty(A)
    C = [C; A]; d = [d; b];
  end
  if ~isempty(lb)
    lb = reshape(lb, n, 1);
    C = [C; -eye(n)]; d = [d; -lb];
  end
  if ~isempty(ub)
    ub = reshape(ub, n, 1);
    C = [C; eye(n)]; d = [d; ub];
  end
  m = size(C, 1);

  % ---------- initial feasible point (bounds-clamped, then fix violations) ---
  xcur = zeros(n, 1);
  if ~isempty(lb), xcur = max(xcur, lb); end
  if ~isempty(ub), xcur = min(xcur, ub); end
  [xcur, ok] = project_feasible(xcur, C, d, 1000, 1e-9);
  if ~ok
    info.status = 'infeasible-start'; info.iter = 0;
    x = zeros(n, 1); obj = inf; info.lambda = [];
    return;
  end

  % project onto the equality manifold once to start cleanly
  if ~isempty(Aeq)
    xcur = project_equality(xcur, Aeq, beq);
    [xcur, ok] = project_feasible(xcur, C, d, 100, 1e-9);
  end

  % ---------- active set over inequality constraints ----------
  W = [];
  for i = 1:m
    if abs(C(i,:) * xcur - d(i)) <= eqtol
      W = [W; i];
    end
  end
  W = unique(W);

  info.iter = 0; info.status = 'maxiter';
  lambda = zeros(m, 1);
  for iter = 1:maxit
    info.iter = iter;
    [xnew, lamW] = eqp_solve(H, f, Aeq, beq, C(W,:), d(W,:), xcur);
    if isempty(xnew)
      [xnew, lamW] = eqp_solve(H + 1e-6 * eye(n), f, Aeq, beq, C(W,:), d(W,:), xcur);
    end
    p = xnew - xcur;
    if norm(p, 'inf') < 1e-10
      % at subproblem optimum: check inequality multipliers (>= 0 required)
      neg = find(lamW < -tol);
      if isempty(neg)
        x = xnew;
        info.status = 'optimal';
        obj = 0.5 * x' * H * x + f' * x;
        lambda(W) = lamW;
        info.lambda = lambda;
        return;
      end
      [~, loc] = min(lamW(neg));
      droppos = neg(loc);         % position within lamW / W
      W(droppos) = [];
      xcur = xnew;
    else
      % line search to next violated inequality constraint
      alpha = 1.0; addrow = -1;
      for i = 1:m
        if any(W == i), continue; end   % already active
        aip = C(i,:) * p;
        if aip > 1e-12
          alphai = (d(i) - C(i,:) * xcur) / aip;
          if alphai < alpha
            alpha = alphai; addrow = i;
          end
        end
      end
      xcur = xcur + alpha * p;
      if addrow > 0 && alpha < 1 - 1e-9
        W = unique([W; addrow]);
      end
    end
  end
  x = xcur;
  obj = 0.5 * x' * H * x + f' * x;
  info.lambda = lambda;
endfunction

% --- project x onto the inequality feasible set (halfspace projections) ---
function [x, ok] = project_feasible(x, C, d, maxit, tol)
  ok = false;
  m = size(C, 1);
  if m == 0, ok = true; return; end
  for it = 1:maxit
    viol = C * x - d;
    [vmax, i] = max(viol);
    if vmax <= tol, ok = true; return; end
    ai = C(i, :);
    anorm2 = ai * ai';
    if anorm2 < 1e-14, continue; end
    x = x - (vmax / anorm2) * ai';  % project onto halfspace a_i x <= d_i
  end
  ok = (max(C * x - d) <= tol);
endfunction

% --- project x onto the equality manifold Aeq x = beq (least-squares) ---
function xp = project_equality(x, Aeq, beq)
  if isempty(Aeq), xp = x; return; end
  % xp = x - Aeq' * pinv(Aeq*Aeq') * (Aeq*x - beq)
  G = Aeq * Aeq';
  xp = x - Aeq' * ((G + 1e-9 * eye(size(G))) \ (Aeq * x - beq));
endfunction

% --- equality-constrained QP subproblem via KKT ---
%   min 1/2 x'Hx + f'x  s.t. Aeq x = beq,  Cw x = dw  (active inequalities)
function [x, lam] = eqp_solve(H, f, Aeq, beq, Cw, dw, xwarm)
  n = length(f);
  if isempty(Aeq), Aeq = zeros(0, n); beq = zeros(0, 1); end
  if isempty(Cw),  Cw = zeros(0, n);  dw = zeros(0, 1); end
  neq = size(Aeq, 1); nr = size(Cw, 1);
  if neq == 0 && nr == 0
    x = -H \ f; lam = [];
    return;
  end
  % KKT: [H  Aeq'  Cw'; Aeq  0   0 ; Cw  0   0 ] [x; leq; lw] = [-f; beq; dw]
  KKT = [H, Aeq', Cw';
         Aeq, zeros(neq), zeros(neq, nr);
         Cw,  zeros(nr, neq), zeros(nr)];
  rhs = [-f; beq; dw];
  sol = KKT \ rhs;
  x = sol(1:n);
  lam = sol(n+1:n+nr);   % inequality multipliers only
endfunction
