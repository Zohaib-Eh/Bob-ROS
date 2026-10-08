import { Info as InfoIcon } from "lucide-react";
import { useId, useState } from "react";
import { GLOSSARY, type Term } from "./glossary";

export default function Info({ term }: { term: Term }) {
  const [open, setOpen] = useState(false);
  const id = useId();
  const [title, text] = GLOSSARY[term];
  return (
    <span className="info" onMouseEnter={() => setOpen(true)} onMouseLeave={() => setOpen(false)}>
      <button type="button" className="info-btn" aria-label={`What is ${title}?`} aria-describedby={open ? id : undefined}
        onFocus={() => setOpen(true)} onBlur={() => setOpen(false)}
        onClick={(e) => { e.stopPropagation(); setOpen((o) => !o); }}>
        <InfoIcon size={13} />
      </button>
      {open && <span role="tooltip" id={id} className="info-card"><strong>{title}</strong>{text}</span>}
    </span>
  );
}
