import { X } from "lucide-react";

export default function Banner({ message, onClose }: { message: string | null; onClose: () => void }) {
  if (!message) return null;
  return (
    <div className="banner" role="alert">
      <span>{message}</span>
      <span className="spacer" />
      <button className="btn btn-ghost btn-icon" aria-label="Dismiss" onClick={onClose}><X size={14} /></button>
    </div>
  );
}
